import argparse
import json
import math
import os
from collections import defaultdict

import yaml


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


def _dist_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _point_in_polygon(x, y, polygon):
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        crosses = (yi > y) != (yj > y)
        if crosses:
            x_intersection = (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi
            if x < x_intersection:
                inside = not inside
        j = i
    return inside


def _aliases_by_label(ontology):
    mapping = defaultdict(list)
    for item in ontology.get('objects', []):
        label = item.get('label', '')
        if label:
            mapping[label] = item.get('aliases', [])
    return mapping


def _region_for_point(point, regions):
    x, y = point[0], point[1]
    for region in regions:
        polygon = region.get('polygon', [])
        if len(polygon) >= 3 and _point_in_polygon(x, y, polygon):
            return region.get('region_id', '')
    return ''


def _pose_from_xyz_yaw(x, y, z, yaw):
    half = yaw * 0.5
    return {
        'position': {'x': x, 'y': y, 'z': z},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(half), 'w': math.cos(half)},
    }


def _stamp_key(stamp):
    return float(stamp.get('sec', 0)) + float(stamp.get('nanosec', 0)) * 1e-9


def _stamp_dict(stamp):
    return {
        'sec': int(stamp.get('sec', 0)),
        'nanosec': int(stamp.get('nanosec', 0)),
    }


def _merge_detection(nodes, detection, association_radius):
    label = detection['label']
    point = detection['map_position']
    candidates = [
        node for node in nodes
        if node['label'] == label and _dist_xy(node['centroid'], point) < association_radius
    ]
    if not candidates:
        node = {
            'label': label,
            'centroid': point[:],
            'reachable_sum': detection['observed_from'][:],
            'confidence_sum': float(detection['confidence']),
            'observations': 1,
            'first_seen': detection['stamp'],
            'last_seen': detection['stamp'],
            'observation_frames': [detection['frame_id']],
        }
        nodes.append(node)
        return

    node = min(candidates, key=lambda item: _dist_xy(item['centroid'], point))
    n = node['observations']
    node['centroid'] = [
        (node['centroid'][0] * n + point[0]) / (n + 1),
        (node['centroid'][1] * n + point[1]) / (n + 1),
        (node['centroid'][2] * n + point[2]) / (n + 1),
    ]
    node['reachable_sum'] = [
        node['reachable_sum'][0] + detection['observed_from'][0],
        node['reachable_sum'][1] + detection['observed_from'][1],
        node['reachable_sum'][2] + detection['observed_from'][2],
    ]
    node['confidence_sum'] += float(detection['confidence'])
    node['observations'] += 1
    if _stamp_key(detection['stamp']) < _stamp_key(node['first_seen']):
        node['first_seen'] = detection['stamp']
    if _stamp_key(detection['stamp']) > _stamp_key(node['last_seen']):
        node['last_seen'] = detection['stamp']
    node['observation_frames'].append(detection['frame_id'])


def _finalize_nodes(nodes, regions, aliases):
    label_counts = defaultdict(int)
    objects = []
    for node in nodes:
        label = node['label']
        label_counts[label] += 1
        suffix = label_counts[label]
        observations = node['observations']
        centroid = node['centroid']
        reachable = [
            node['reachable_sum'][0] / observations,
            node['reachable_sum'][1] / observations,
            node['reachable_sum'][2] / observations,
        ]
        region_id = _region_for_point(centroid, regions)
        objects.append({
            'object_id': f'{label}_{suffix}',
            'label': label,
            'aliases': aliases.get(label, []),
            'object_pose_map': _pose_from_xyz_yaw(centroid[0], centroid[1], centroid[2], 0.0),
            'reachable_pose_map': _pose_from_xyz_yaw(reachable[0], reachable[1], 0.0, reachable[2]),
            'region_id': region_id,
            'confidence': node['confidence_sum'] / observations,
            'observations': observations,
            'first_seen': _stamp_dict(node['first_seen']),
            'last_seen': _stamp_dict(node['last_seen']),
            'observation_frames': node['observation_frames'],
        })
    return objects


def _compute_near_relations(objects, threshold):
    relations = []
    for i, left in enumerate(objects):
        for right in objects[i + 1:]:
            if left.get('region_id') and left.get('region_id') != right.get('region_id'):
                continue
            a = left['object_pose_map']['position']
            b = right['object_pose_map']['position']
            distance = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            if distance <= threshold:
                relations.append({
                    'type': 'near',
                    'source': left['object_id'],
                    'target': right['object_id'],
                    'distance_m': distance,
                })
                relations.append({
                    'type': 'near',
                    'source': right['object_id'],
                    'target': left['object_id'],
                    'distance_m': distance,
                })
    return relations


def main():
    parser = argparse.ArgumentParser(description='Fuse grounded LyraGraph detections into graph.json.')
    parser.add_argument('--detections', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--regions', default='config/lyragraph/home_regions.yaml')
    parser.add_argument('--ontology', default='config/lyragraph/ontology.yaml')
    parser.add_argument('--run-id', default='default')
    parser.add_argument('--association-radius', type=float, default=0.75)
    parser.add_argument('--near-threshold', type=float, default=1.5)
    args = parser.parse_args()

    detections = _load_jsonl(args.detections)
    regions_doc = _load_yaml(args.regions, {'regions': []})
    ontology = _load_yaml(args.ontology, {'objects': []})
    aliases = _aliases_by_label(ontology)

    nodes = []
    for detection in detections:
        _merge_detection(nodes, detection, args.association_radius)

    regions = regions_doc.get('regions', [])
    objects = _finalize_nodes(nodes, regions, aliases)
    graph = {
        'schema_version': '0.1',
        'run_id': args.run_id,
        'objects': objects,
        'regions': regions,
        'relations': _compute_near_relations(objects, args.near_threshold),
        'metadata': {
            'source_detections': os.path.abspath(args.detections),
            'association_radius_m': args.association_radius,
            'near_threshold_m': args.near_threshold,
        },
    }

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(graph, stream, indent=2)
    print(f'Wrote graph with {len(objects)} objects to {args.output}')


if __name__ == '__main__':
    main()
