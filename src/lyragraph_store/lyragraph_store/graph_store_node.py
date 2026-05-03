import json
import math
import os

import rclpy
from geometry_msgs.msg import Point, Point32, Polygon, Pose
from lyragraph_msgs.msg import GraphObject, GraphObjectArray, GraphRegion, GraphRegionArray
from lyragraph_msgs.srv import QueryGraph
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def _norm(text):
    return str(text or '').strip().lower().replace(' ', '_')


def _time_msg(value):
    from builtin_interfaces.msg import Time
    stamp = Time()
    if isinstance(value, dict):
        stamp.sec = int(value.get('sec', 0))
        stamp.nanosec = int(value.get('nanosec', 0))
    return stamp


def _pose_msg(value):
    pose = Pose()
    if isinstance(value, dict):
        pos = value.get('position', {})
        ori = value.get('orientation', {})
        pose.position.x = float(pos.get('x', 0.0))
        pose.position.y = float(pos.get('y', 0.0))
        pose.position.z = float(pos.get('z', 0.0))
        pose.orientation.x = float(ori.get('x', 0.0))
        pose.orientation.y = float(ori.get('y', 0.0))
        pose.orientation.z = float(ori.get('z', 0.0))
        pose.orientation.w = float(ori.get('w', 1.0))
    return pose


def _polygon_msg(points):
    polygon = Polygon()
    for point in points or []:
        p = Point32()
        p.x = float(point[0])
        p.y = float(point[1])
        p.z = 0.0
        polygon.points.append(p)
    return polygon


def _label_matches(obj, label):
    label = _norm(label)
    if not label:
        return True
    names = [_norm(obj.get('label', ''))]
    names.extend(_norm(alias) for alias in obj.get('aliases', []))
    return label in names


class GraphStore(Node):
    def __init__(self):
        super().__init__('lyragraph_store')
        self.declare_parameter('graph_file', 'data/lyragraph_graphs/default/graph.json')
        self.graph_file = self.get_parameter('graph_file').value
        self.graph = self._load_graph(self.graph_file)
        self.objects = self.graph.get('objects', [])
        self.regions = self.graph.get('regions', [])
        self.relations = self.graph.get('relations', [])

        self.object_pub = self.create_publisher(GraphObjectArray, '/lyragraph/objects', 10)
        self.region_pub = self.create_publisher(GraphRegionArray, '/lyragraph/regions', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lyragraph/markers', 10)
        self.create_service(QueryGraph, '/lyragraph/query_graph', self._on_query)
        self.create_timer(1.0, self._publish_graph)

        self.get_logger().info(
            f'Loaded graph {self.graph_file}: {len(self.objects)} objects, {len(self.regions)} regions'
        )

    def _load_graph(self, path):
        if not os.path.exists(path):
            self.get_logger().warn(f'Graph file does not exist yet: {path}')
            return {'schema_version': '0.1', 'objects': [], 'regions': [], 'relations': []}
        with open(path, 'r', encoding='utf-8') as stream:
            return json.load(stream)

    def _object_to_msg(self, obj):
        msg = GraphObject()
        msg.object_id = str(obj.get('object_id', ''))
        msg.label = str(obj.get('label', ''))
        msg.aliases = [str(alias) for alias in obj.get('aliases', [])]
        msg.object_pose_map = _pose_msg(obj.get('object_pose_map', {}))
        msg.reachable_pose_map = _pose_msg(obj.get('reachable_pose_map', {}))
        msg.region_id = str(obj.get('region_id', ''))
        msg.confidence = float(obj.get('confidence', 0.0))
        msg.observations = int(obj.get('observations', 0))
        msg.first_seen = _time_msg(obj.get('first_seen', {}))
        msg.last_seen = _time_msg(obj.get('last_seen', {}))
        return msg

    def _region_to_msg(self, region):
        msg = GraphRegion()
        msg.region_id = str(region.get('region_id', ''))
        msg.label = str(region.get('label', ''))
        msg.polygon_map = _polygon_msg(region.get('polygon', []))
        msg.policy_default = str(region.get('policy_default', 'neutral'))
        msg.default_cost = float(region.get('default_cost', 0.0))
        return msg

    def _header(self):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id='map')

    def _publish_graph(self):
        object_array = GraphObjectArray()
        object_array.header = self._header()
        object_array.objects = [self._object_to_msg(obj) for obj in self.objects]
        self.object_pub.publish(object_array)

        region_array = GraphRegionArray()
        region_array.header = self._header()
        region_array.regions = [self._region_to_msg(region) for region in self.regions]
        self.region_pub.publish(region_array)

        self.marker_pub.publish(self._markers())

    def _markers(self):
        markers = MarkerArray()
        marker_id = 0
        for obj in self.objects:
            pose = _pose_msg(obj.get('object_pose_map', {}))
            marker = Marker()
            marker.header = self._header()
            marker.ns = 'lyragraph_objects'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.r = 0.1
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 0.85
            markers.markers.append(marker)

            text = Marker()
            text.header = self._header()
            text.ns = 'lyragraph_labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = pose
            text.pose.position.z += 0.35
            text.scale.z = 0.18
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = obj.get('object_id', obj.get('label', 'object'))
            markers.markers.append(text)

        for region in self.regions:
            polygon = region.get('polygon', [])
            if len(polygon) < 3:
                continue
            marker = Marker()
            marker.header = self._header()
            marker.ns = 'lyragraph_regions'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.1
            marker.color.a = 0.9
            closed = polygon + [polygon[0]]
            for xy in closed:
                point = Point()
                point.x = float(xy[0])
                point.y = float(xy[1])
                point.z = 0.03
                marker.points.append(point)
            markers.markers.append(marker)
        return markers

    def _on_query(self, request, response):
        target_label = _norm(request.target_label)
        region_label = _norm(request.region_label)
        near_label = _norm(request.near_label)

        matches = [obj for obj in self.objects if _label_matches(obj, target_label)]
        if region_label:
            region_ids = {
                region.get('region_id', '')
                for region in self.regions
                if _norm(region.get('label', '')) == region_label or _norm(region.get('region_id', '')) == region_label
            }
            matches = [obj for obj in matches if obj.get('region_id', '') in region_ids]

        if near_label:
            near_ids = {obj.get('object_id', '') for obj in self.objects if _label_matches(obj, near_label)}
            allowed_sources = {
                relation.get('source', '')
                for relation in self.relations
                if relation.get('type') == 'near' and relation.get('target', '') in near_ids
            }
            matches = [obj for obj in matches if obj.get('object_id', '') in allowed_sources]

        response.success = bool(matches)
        response.message = f'{len(matches)} match(es)' if matches else 'no matching graph object'
        response.matches = [self._object_to_msg(obj) for obj in matches]
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraphStore()
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
