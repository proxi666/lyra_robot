import json
import math
import os
import struct
import time
import zlib

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


def _stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _stamp_to_dict(stamp):
    return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}


def _transform_to_pose_dict(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    return {
        'position': {'x': t.x, 'y': t.y, 'z': t.z},
        'orientation': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w},
    }


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _png_chunk(name, data):
    payload = name + data
    return (
        struct.pack('>I', len(data)) +
        payload +
        struct.pack('>I', zlib.crc32(payload) & 0xffffffff)
    )


def _write_rgb_png(path, msg):
    if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        raise ValueError(f'Unsupported RGB encoding: {msg.encoding}')

    channels = 4 if msg.encoding in ('rgba8', 'bgra8') else 3
    rows = []
    for y in range(msg.height):
        start = y * msg.step
        row = msg.data[start:start + msg.width * channels]
        if msg.encoding in ('bgr8', 'bgra8'):
            converted = bytearray()
            for i in range(0, len(row), channels):
                converted.extend([row[i + 2], row[i + 1], row[i]])
            row = bytes(converted)
        elif channels == 4:
            converted = bytearray()
            for i in range(0, len(row), channels):
                converted.extend([row[i], row[i + 1], row[i + 2]])
            row = bytes(converted)
        rows.append(b'\x00' + bytes(row))

    raw = b''.join(rows)
    header = struct.pack('>IIBBBBB', msg.width, msg.height, 8, 2, 0, 0, 0)
    with open(path, 'wb') as out:
        out.write(b'\x89PNG\r\n\x1a\n')
        out.write(_png_chunk(b'IHDR', header))
        out.write(_png_chunk(b'IDAT', zlib.compress(raw)))
        out.write(_png_chunk(b'IEND', b''))


def _depth_to_array(msg):
    if msg.encoding != '32FC1':
        raise ValueError(f'Unsupported depth encoding: {msg.encoding}')
    dtype = '>f4' if msg.is_bigendian else '<f4'
    values_per_row = msg.step // 4
    data = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, values_per_row))
    return data[:, :msg.width].astype(np.float32, copy=True)


class KeyframeLogger(Node):
    def __init__(self):
        super().__init__('lyragraph_keyframe_logger')

        self.declare_parameter('run_id', 'default')
        self.declare_parameter('output_root', 'data/lyragraph_runs')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('min_time_delta_s', 0.5)
        self.declare_parameter('min_translation_delta_m', 0.25)
        self.declare_parameter('min_yaw_delta_deg', 12.0)
        self.declare_parameter('max_depth_rgb_delta_s', 0.25)

        run_id = self.get_parameter('run_id').get_parameter_value().string_value
        output_root = self.get_parameter('output_root').get_parameter_value().string_value
        if run_id == 'default':
            run_id = time.strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(output_root, run_id)
        self.rgb_dir = os.path.join(self.run_dir, 'rgb')
        self.depth_dir = os.path.join(self.run_dir, 'depth')
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        self.manifest_path = os.path.join(self.run_dir, 'manifest.jsonl')

        self.depth_msg = None
        self.camera_info_msg = None
        self.last_saved = None
        self.frame_index = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._on_camera_info,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self._on_depth,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter('rgb_topic').value,
            self._on_rgb,
            10,
        )

        self.get_logger().info(f'Logging LyraGraph keyframes to {self.run_dir}')

    def _on_camera_info(self, msg):
        self.camera_info_msg = msg

    def _on_depth(self, msg):
        self.depth_msg = msg

    def _on_rgb(self, msg):
        if self.depth_msg is None or self.camera_info_msg is None:
            return

        rgb_time = _stamp_to_float(msg.header.stamp)
        depth_time = _stamp_to_float(self.depth_msg.header.stamp)
        if abs(rgb_time - depth_time) > float(self.get_parameter('max_depth_rgb_delta_s').value):
            return

        try:
            map_frame = self.get_parameter('map_frame').value
            base_frame = self.get_parameter('base_frame').value
            camera_frame = self.get_parameter('camera_frame').value
            camera_tf = self.tf_buffer.lookup_transform(
                map_frame,
                camera_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.2),
            )
            robot_tf = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Skipping keyframe without TF: {exc}', throttle_duration_sec=5.0)
            return

        if not self._should_save(rgb_time, robot_tf):
            return

        frame_id = f'frame_{self.frame_index:06d}'
        rgb_name = f'{frame_id}.png'
        depth_name = f'{frame_id}.npy'
        rgb_path = os.path.join(self.rgb_dir, rgb_name)
        depth_path = os.path.join(self.depth_dir, depth_name)

        try:
            _write_rgb_png(rgb_path, msg)
            np.save(depth_path, _depth_to_array(self.depth_msg))
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f'Failed to save keyframe {frame_id}: {exc}')
            return

        cam = self.camera_info_msg
        record = {
            'frame_id': frame_id,
            'stamp': _stamp_to_dict(msg.header.stamp),
            'rgb_path': os.path.join('rgb', rgb_name),
            'depth_path': os.path.join('depth', depth_name),
            'rgb_encoding': msg.encoding,
            'depth_encoding': self.depth_msg.encoding,
            'width': int(msg.width),
            'height': int(msg.height),
            'camera_frame': self.get_parameter('camera_frame').value,
            'base_frame': self.get_parameter('base_frame').value,
            'map_frame': self.get_parameter('map_frame').value,
            'intrinsics': {
                'fx': float(cam.k[0]),
                'fy': float(cam.k[4]),
                'cx': float(cam.k[2]),
                'cy': float(cam.k[5]),
            },
            'camera_pose_map': _transform_to_pose_dict(camera_tf),
            'robot_pose_map': _transform_to_pose_dict(robot_tf),
        }
        with open(self.manifest_path, 'a', encoding='utf-8') as manifest:
            manifest.write(json.dumps(record) + '\n')

        self.frame_index += 1
        self.last_saved = {
            'time': rgb_time,
            'x': robot_tf.transform.translation.x,
            'y': robot_tf.transform.translation.y,
            'yaw': _yaw_from_quaternion(robot_tf.transform.rotation),
        }
        self.get_logger().info(f'Saved keyframe {frame_id}')

    def _should_save(self, stamp_s, robot_tf):
        if self.last_saved is None:
            return True

        min_dt = float(self.get_parameter('min_time_delta_s').value)
        min_dist = float(self.get_parameter('min_translation_delta_m').value)
        min_yaw = math.radians(float(self.get_parameter('min_yaw_delta_deg').value))

        x = robot_tf.transform.translation.x
        y = robot_tf.transform.translation.y
        yaw = _yaw_from_quaternion(robot_tf.transform.rotation)
        dist = math.hypot(x - self.last_saved['x'], y - self.last_saved['y'])
        yaw_delta = abs(_angle_delta(yaw, self.last_saved['yaw']))
        dt = stamp_s - self.last_saved['time']
        return dt >= min_dt or dist >= min_dist or yaw_delta >= min_yaw


def main(args=None):
    rclpy.init(args=args)
    node = KeyframeLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
