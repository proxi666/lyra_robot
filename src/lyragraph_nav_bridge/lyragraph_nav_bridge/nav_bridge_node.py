import math
import re

import rclpy
from geometry_msgs.msg import PoseStamped
from lyragraph_msgs.srv import QueryGraph, ResolveSemanticGoal
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def _clean(text):
    return re.sub(r'\s+', ' ', str(text or '').strip().lower())


def _phrase(text):
    text = _clean(text)
    text = re.sub(r'^(the|a|an)\s+', '', text)
    return text.replace(' ', '_')


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _pose_distance(a, b):
    return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)


class NavBridge(Node):
    def __init__(self):
        super().__init__('lyragraph_nav_bridge')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('graph_query_service', '/lyragraph/query_graph')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.query_client = self.create_client(QueryGraph, self.get_parameter('graph_query_service').value)
        self.compute_path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_service(ResolveSemanticGoal, '/lyragraph/resolve_semantic_goal', self._on_resolve)

        self.get_logger().info('LyraGraph Nav2 bridge ready')

    def _parse_instruction(self, instruction):
        text = _clean(instruction)
        patterns = [
            (r'^go to (?:the )?(?P<target>.+?) near (?:the )?(?P<near>.+)$', 'near'),
            (r'^go to (?:the )?(?P<target>.+?) in (?:the )?(?P<region>.+)$', 'region'),
            (r'^go to (?:the )?(?P<target>.+)$', 'target'),
        ]
        for pattern, mode in patterns:
            match = re.match(pattern, text)
            if not match:
                continue
            groups = match.groupdict()
            return {
                'target_label': _phrase(groups.get('target', '')),
                'region_label': _phrase(groups.get('region', '')),
                'near_label': _phrase(groups.get('near', '')),
                'mode': mode,
            }
        return None

    def _current_pose(self):
        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        tf = self.tf_buffer.lookup_transform(
            map_frame,
            base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        pose = PoseStamped()
        pose.header.frame_id = map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _query_graph(self, parsed):
        if not self.query_client.wait_for_service(timeout_sec=2.0):
            return None, 'graph query service is not available'
        request = QueryGraph.Request()
        request.target_label = parsed['target_label']
        request.region_label = parsed['region_label']
        request.near_label = parsed['near_label']
        future = self.query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return None, 'graph query timed out'
        result = future.result()
        if not result.success:
            return None, result.message
        return result.matches, result.message

    def _choose_target(self, matches, current_pose):
        def score(obj):
            dist = _pose_distance(obj.reachable_pose_map, current_pose.pose)
            return (-float(obj.confidence), dist)
        return sorted(matches, key=score)[0]

    def _goal_from_object(self, obj):
        goal = PoseStamped()
        goal.header.frame_id = self.get_parameter('map_frame').value
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = obj.reachable_pose_map
        return goal

    def _validate_path(self, goal_pose):
        if not self.compute_path_client.wait_for_server(timeout_sec=3.0):
            return False, 'ComputePathToPose action server is not available'

        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.planner_id = self.get_parameter('planner_id').value
        goal.use_start = False

        send_future = self.compute_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            return False, 'ComputePathToPose goal request timed out'
        handle = send_future.result()
        if not handle.accepted:
            return False, 'ComputePathToPose rejected the candidate goal'

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        if not result_future.done() or result_future.result() is None:
            return False, 'ComputePathToPose result timed out'
        path = result_future.result().result.path
        if not path.poses:
            return False, 'no safe pose found'
        return True, f'validated path with {len(path.poses)} poses'

    def _send_navigation(self, goal_pose):
        if not self.navigate_client.wait_for_server(timeout_sec=3.0):
            return False, 'NavigateToPose action server is not available'
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        send_future = self.navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            return False, 'NavigateToPose goal request timed out'
        handle = send_future.result()
        if not handle.accepted:
            return False, 'NavigateToPose rejected the goal'
        return True, 'goal accepted by Nav2'

    def _on_resolve(self, request, response):
        parsed = self._parse_instruction(request.instruction)
        if parsed is None:
            response.success = False
            response.message = 'unsupported instruction; use "go to the object", "go to the object in the region", or "go to the object near the object"'
            return response

        try:
            current_pose = self._current_pose()
        except TransformException as exc:
            response.success = False
            response.message = f'current robot pose unavailable: {exc}'
            return response

        matches, message = self._query_graph(parsed)
        if not matches:
            response.success = False
            response.message = message
            return response

        target = self._choose_target(matches, current_pose)
        goal_pose = self._goal_from_object(target)
        valid, valid_message = self._validate_path(goal_pose)
        if not valid:
            response.success = False
            response.message = valid_message
            response.target = target
            response.goal_pose_map = goal_pose
            return response

        response.target = target
        response.goal_pose_map = goal_pose
        if request.execute:
            sent, sent_message = self._send_navigation(goal_pose)
            response.success = sent
            response.message = sent_message if sent else f'path valid, navigation failed: {sent_message}'
            return response

        yaw = _yaw_from_quaternion(goal_pose.pose.orientation)
        response.success = True
        response.message = (
            f'{valid_message}; resolved {target.object_id} '
            f'to ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}, {yaw:.2f})'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NavBridge()
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
