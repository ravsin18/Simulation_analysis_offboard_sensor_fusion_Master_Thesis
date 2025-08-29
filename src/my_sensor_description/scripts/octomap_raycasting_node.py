#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid  # Optional

from octomap import OcTree  # You must have python-octomap bindings installed (octomap-python)
import math

class OctomapRaycastingNode(Node):
    def __init__(self):
        super().__init__('octomap_raycasting_node')

        self.declare_parameter('input_topics', [])
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('resolution', 0.1)

        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.octree = OcTree(self.resolution)

        self.subs = []
        for topic in self.input_topics:
            sub = self.create_subscription(PointCloud2, topic, self.make_callback(topic), 10)
            self.subs.append(sub)

        self.octomap_pub = self.create_publisher(Octomap, '/octomap_custom', 10)
        self.create_timer(2.0, self.publish_octomap)

    def make_callback(self, topic_name):
        def callback(msg):
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )
                transformed_cloud = do_transform_cloud(msg, trans)

                sensor_origin = [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ]

                points = pc2.read_points(transformed_cloud, field_names=["x", "y", "z"], skip_nans=True)

                for point in points:
                    endpoint = (point[0], point[1], point[2])
                    origin = tuple(sensor_origin)
                    self.octree.insertRay(origin, endpoint)

                self.get_logger().info(f'Inserted {len(list(points))} points from {topic_name}')

            except Exception as e:
                self.get_logger().warn(f"[{topic_name}] Transform or raycast failed: {e}")
        return callback

    def publish_octomap(self):
        msg = Octomap()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame

        binary_data = self.octree.writeBinary()
        msg.binary = True
        msg.id = "OcTree"
        msg.resolution = self.octree.getResolution()
        msg.data = list(binary_data)

        self.octomap_pub.publish(msg)
        self.get_logger().info('Published OctoMap with ray casting')


def main(args=None):
    rclpy.init(args=args)
    node = OctomapRaycastingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
