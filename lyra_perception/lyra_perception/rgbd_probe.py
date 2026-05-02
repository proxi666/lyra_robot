import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class RgbdProbe(Node):
    def __init__(self):
        super().__init__('rgbd_probe_node')

        self.rgb_msg = None
        self.depth_msg = None
        self.camera_info_msg = None

        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._on_rgb,
            10,
        )
        self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self._on_depth,
            10,
        )
        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self._on_camera_info,
            10,
        )

        self.summary_pub = self.create_publisher(
            String,
            '/lyra/perception/rgbd_probe',
            10,
        )
        self.create_timer(1.0, self._publish_summary)

    def _on_rgb(self, msg):
        self.rgb_msg = msg

    def _on_depth(self, msg):
        self.depth_msg = msg

    def _on_camera_info(self, msg):
        self.camera_info_msg = msg

    def _center_depth_meters(self):
        if self.depth_msg is None:
            return 'unavailable'

        msg = self.depth_msg
        if msg.encoding != '32FC1':
            return f'unsupported encoding {msg.encoding}'

        center_x = msg.width // 2
        center_y = msg.height // 2
        byte_offset = center_y * msg.step + center_x * 4

        if byte_offset + 4 > len(msg.data):
            return 'unavailable'

        fmt = '>f' if msg.is_bigendian else '<f'
        depth = struct.unpack_from(fmt, msg.data, byte_offset)[0]

        if math.isnan(depth):
            return 'nan'
        if math.isinf(depth):
            return 'inf'
        return f'{depth:.3f}'

    def _publish_summary(self):
        if self.rgb_msg is None or self.depth_msg is None or self.camera_info_msg is None:
            self.get_logger().info('Waiting for RGB image, depth image, and camera info...')
            return

        rgb = self.rgb_msg
        depth = self.depth_msg
        camera_info = self.camera_info_msg

        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        summary = (
            f'rgb={rgb.width}x{rgb.height} {rgb.encoding} frame={rgb.header.frame_id}; '
            f'depth={depth.width}x{depth.height} {depth.encoding} frame={depth.header.frame_id}; '
            f'center_depth_m={self._center_depth_meters()}; '
            f'intrinsics=fx:{fx:.3f}, fy:{fy:.3f}, cx:{cx:.3f}, cy:{cy:.3f}'
        )

        self.get_logger().info(summary)
        self.summary_pub.publish(String(data=summary))


def main(args=None):
    rclpy.init(args=args)
    node = RgbdProbe()
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
