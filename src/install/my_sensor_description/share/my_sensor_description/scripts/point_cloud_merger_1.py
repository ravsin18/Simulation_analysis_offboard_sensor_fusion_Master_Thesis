#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from rclpy.time import Time

class PointCloudMerger(Node):
def init(self):
super().init('point_cloud_merger')

    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    self.cloud1 = None
    self.cloud2 = None

    self.target_frame = 'map'

    self.sub1 = self.create_subscription(
        PointCloud2,
        '/camera_robot/camera_robot_sensor/points',
        self.callback1,
        10
    )

    self.sub2 = self.create_subscription(
        PointCloud2,
        '/camera_robot_1/camera_robot_sensor_1/points',
        self.callback2,
        10
    )

    self.pub = self.create_publisher(PointCloud2, '/merged_camera_points', 10)

def callback1(self, msg):
    try:
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            msg.header.frame_id,
            Time()
        )
        self.cloud1 = do_transform_cloud(msg, transform)
        self.merge_and_publish()
    except Exception as e:
        self.get_logger().warn(f'Failed to transform cloud1: {e}')

def callback2(self, msg):
    try:
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            msg.header.frame_id,
            Time()
        )
        self.cloud2 = do_transform_cloud(msg, transform)
        self.merge_and_publish()
    except Exception as e:
        self.get_logger().warn(f'Failed to transform cloud2: {e}')

def merge_and_publish(self):
    if self.cloud1 is None or self.cloud2 is None:
        return

    try:
        points1 = list(pc2.read_points(self.cloud1, skip_nans=True))
        points2 = list(pc2.read_points(self.cloud2, skip_nans=True))

        merged_points = points1 + points2
        merged_cloud = pc2.create_cloud(self.cloud1.header, self.cloud1.fields, merged_points)

        self.pub.publish(merged_cloud)
    except Exception as e:
        self.get_logger().warn(f'Failed to merge point clouds: {e}')

def main(args=None):
rclpy.init(args=args)
node = PointCloudMerger()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

if name == 'main':
main()
