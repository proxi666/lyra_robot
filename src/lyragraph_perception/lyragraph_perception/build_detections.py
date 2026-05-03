import argparse
import json
import math
import os
import re

import numpy as np
import yaml


DEFAULT_LABELS = [
    'sofa',
    'chair',
    'table',
    'fridge',
    'door',
    'charging_dock',
    'bed',
    'desk',
    'tv',
    'plant',
]


def _load_jsonl(path):
    rows = []
    with open(path, 'r', encoding='utf-8') as stream:
        for line in stream:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _load_yaml(path, default):
    if not path or not os.path.exists(path):
        return default
    with open(path, 'r', encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    return value if value is not None else default


def _quat_rotate(q, point):
    x, y, z = point
    qx = q['x']
    qy = q['y']
    qz = q['z']
    qw = q['w']

    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z

    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    )


def _transform_point(pose, point):
    rotated = _quat_rotate(pose['orientation'], point)
    pos = pose['position']
    return [
        rotated[0] + pos['x'],
        rotated[1] + pos['y'],
        rotated[2] + pos['z'],
    ]


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q['w'] * q['z'] + q['x'] * q['y'])
    cosy_cosp = 1.0 - 2.0 * (q['y'] * q['y'] + q['z'] * q['z'])
    return math.atan2(siny_cosp, cosy_cosp)


def _bbox_to_pixels(bbox, width, height):
    if len(bbox) != 4:
        raise ValueError('bbox must have four numbers')
    values = [float(v) for v in bbox]
    max_value = max(values)
    if max_value <= 1.0:
        x1, y1, x2, y2 = values
        values = [x1 * width, y1 * height, x2 * width, y2 * height]
    elif max_value <= 1000.0 and (values[2] > width or values[3] > height):
        x1, y1, x2, y2 = values
        values = [x1 * width / 1000.0, y1 * height / 1000.0, x2 * width / 1000.0, y2 * height / 1000.0]

    x1, y1, x2, y2 = values
    x1 = max(0, min(width - 1, x1))
    x2 = max(0, min(width - 1, x2))
    y1 = max(0, min(height - 1, y1))
    y2 = max(0, min(height - 1, y2))
    return [x1, y1, x2, y2]


def _median_depth_at_bbox(depth, bbox, window_px, min_depth, max_depth):
    height, width = depth.shape
    x1, y1, x2, y2 = _bbox_to_pixels(bbox, width, height)
    cx = int(round((x1 + x2) / 2.0))
    cy = int(round((y1 + y2) / 2.0))
    half = max(1, int(window_px) // 2)
    patch = depth[max(0, cy - half):min(height, cy + half + 1), max(0, cx - half):min(width, cx + half + 1)]
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid >= min_depth) & (valid <= max_depth)]
    if valid.size == 0:
        return None, cx, cy
    return float(np.median(valid)), cx, cy


def _back_project(pixel_x, pixel_y, depth_m, intrinsics):
    fx = intrinsics['fx']
    fy = intrinsics['fy']
    cx = intrinsics['cx']
    cy = intrinsics['cy']
    return [
        (pixel_x - cx) * depth_m / fx,
        (pixel_y - cy) * depth_m / fy,
        depth_m,
    ]


def _extract_json_array(text):
    match = re.search(r'\[[\s\S]*\]', text)
    if not match:
        return []
    return json.loads(match.group(0))


class QwenExtractor:
    def __init__(self, model_name, labels):
        try:
            from PIL import Image
            from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration
        except ImportError as exc:
            raise RuntimeError(
                'Qwen backend requires pillow and transformers with Qwen2.5-VL support. '
                'Use --mock-detections for pipeline testing before model setup.'
            ) from exc

        self.image_cls = Image
        self.processor = AutoProcessor.from_pretrained(model_name)
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            model_name,
            torch_dtype='auto',
            device_map='auto',
        )
        self.labels = labels

    def detect(self, image_path):
        labels_text = ', '.join(self.labels)
        prompt = (
            'Detect visible indoor objects from this ontology only: '
            f'{labels_text}. Return JSON only as a list of objects. '
            'Each object must have label, confidence, and bbox as [x1,y1,x2,y2] in pixel coordinates. '
            'If no object is visible, return [].'
        )
        image = self.image_cls.open(image_path).convert('RGB')
        messages = [{
            'role': 'user',
            'content': [
                {'type': 'image', 'image': image},
                {'type': 'text', 'text': prompt},
            ],
        }]
        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        inputs = self.processor(text=[text], images=[image], return_tensors='pt').to(self.model.device)
        output_ids = self.model.generate(**inputs, max_new_tokens=512)
        generated = self.processor.batch_decode(output_ids[:, inputs.input_ids.shape[1]:], skip_special_tokens=True)[0]
        return _extract_json_array(generated)


def _load_mock_detections(path):
    if not path:
        return {}
    with open(path, 'r', encoding='utf-8') as stream:
        value = json.load(stream)
    if isinstance(value, dict):
        return value
    by_frame = {}
    for item in value:
        by_frame.setdefault(item['frame_id'], []).append(item)
    return by_frame


def _ground_detection(frame, detection, run_dir, config):
    label = str(detection.get('label', '')).strip().lower()
    if not label:
        return None

    bbox = detection.get('bbox')
    if not bbox:
        return None

    depth = np.load(os.path.join(run_dir, frame['depth_path']))
    depth_m, pixel_x, pixel_y = _median_depth_at_bbox(
        depth,
        bbox,
        config['depth_window_px'],
        config['min_depth_m'],
        config['max_depth_m'],
    )
    if depth_m is None:
        return None

    camera_point = _back_project(pixel_x, pixel_y, depth_m, frame['intrinsics'])
    map_point = _transform_point(frame['camera_pose_map'], camera_point)
    robot_pose = frame['robot_pose_map']

    return {
        'frame_id': frame['frame_id'],
        'stamp': frame['stamp'],
        'label': label,
        'confidence': float(detection.get('confidence', 0.5)),
        'bbox': _bbox_to_pixels(bbox, int(frame['width']), int(frame['height'])),
        'pixel': [pixel_x, pixel_y],
        'depth_m': depth_m,
        'camera_point': camera_point,
        'map_position': map_point,
        'observed_from': [
            float(robot_pose['position']['x']),
            float(robot_pose['position']['y']),
            _yaw_from_quaternion(robot_pose['orientation']),
        ],
    }


def main():
    parser = argparse.ArgumentParser(description='Build LyraGraph grounded detections from logged RGB-D keyframes.')
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--output', default=None)
    parser.add_argument('--ontology', default='config/lyragraph/ontology.yaml')
    parser.add_argument('--config', default='config/lyragraph/perception.yaml')
    parser.add_argument('--mock-detections', default=None)
    args = parser.parse_args()

    manifest_path = os.path.join(args.run_dir, 'manifest.jsonl')
    frames = _load_jsonl(manifest_path)
    ontology = _load_yaml(args.ontology, {'objects': [{'label': label, 'aliases': []} for label in DEFAULT_LABELS]})
    labels = [item['label'] for item in ontology.get('objects', [])]

    default_config = {
        'model_name': 'Qwen/Qwen2.5-VL-3B-Instruct',
        'confidence_threshold': 0.25,
        'depth_window_px': 9,
        'min_depth_m': 0.1,
        'max_depth_m': 8.0,
    }
    config = default_config
    config.update(_load_yaml(args.config, {}))

    mock = _load_mock_detections(args.mock_detections)
    extractor = None
    if not mock:
        extractor = QwenExtractor(config['model_name'], labels)

    output_path = args.output or os.path.join(args.run_dir, 'detections.jsonl')
    count = 0
    with open(output_path, 'w', encoding='utf-8') as out:
        for frame in frames:
            image_path = os.path.join(args.run_dir, frame['rgb_path'])
            raw_detections = mock.get(frame['frame_id'], []) if mock else extractor.detect(image_path)
            for detection in raw_detections:
                if float(detection.get('confidence', 0.5)) < float(config['confidence_threshold']):
                    continue
                grounded = _ground_detection(frame, detection, args.run_dir, config)
                if grounded is None:
                    continue
                out.write(json.dumps(grounded) + '\n')
                count += 1
    print(f'Wrote {count} grounded detections to {output_path}')


if __name__ == '__main__':
    main()
