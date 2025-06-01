#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import struct
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('point_cloud_merger')

        # Transform listener setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Define topics to subscribe to
        self.topic_list = [
            '/camera_robot/camera_robot_sensor/points',
            '/camera_robot_1/camera_robot_sensor_1/points',
            '/lidar_robot/lidar_robot_scan/out'
        ]

        self.target_frame = 'map'
        self.clouds = {}  # Latest cloud from each topic
        self._subs = []   # Avoid naming conflict with Node internals

        # Subscribe to topics
        for topic in self.topic_list:
            self.clouds[topic] = None
            sub = self.create_subscription(PointCloud2, topic, self.make_callback(topic), 10)
            self._subs.append(sub)

        # Publisher for merged cloud
        self.pub = self.create_publisher(PointCloud2, '/merged_sensor_points', 10)

    def make_callback(self, topic_name):
        def callback(msg):
            try:
                stripped = self.strip_to_xyz(msg)
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )
                transformed_cloud = do_transform_cloud(stripped, transform)
                self.clouds[topic_name] = self.sanitize_xyz(transformed_cloud)
                self.merge_and_publish()
            except Exception as e:
                self.get_logger().warn(f'Failed to transform cloud from {topic_name}: {e}')
        return callback

    def strip_to_xyz(self, cloud):
        points = list(pc2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True))
        return pc2.create_cloud_xyz32(cloud.header, points)

    def sanitize_xyz(self, cloud):
        header = cloud.header
        xyz_iter = pc2.read_points(cloud, field_names=['x', 'y', 'z'], skip_nans=True)

        new_cloud = PointCloud2()
        new_cloud.header = header
        new_cloud.height = 1
        points = list(xyz_iter)
        new_cloud.width = len(points)
        new_cloud.is_dense = cloud.is_dense
        new_cloud.is_bigendian = cloud.is_bigendian

        new_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        new_cloud.point_step = 12
        new_cloud.row_step = new_cloud.point_step * new_cloud.width

        buf = bytearray(new_cloud.row_step)
        for i, (x, y, z) in enumerate(points):
            struct.pack_into('fff', buf, i * 12, x, y, z)
        new_cloud.data = bytes(buf)

        return new_cloud

    def merge_and_publish(self):
        if not all(self.clouds.values()):
            return  # Wait until all sensors have published

        try:
            all_points = []
            for cloud in self.clouds.values():
                points = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
                all_points.extend(points)

            merged_cloud = pc2.create_cloud_xyz32(next(iter(self.clouds.values())).header, all_points)
            self.pub.publish(merged_cloud)
        except Exception as e:
            self.get_logger().error(f"Failed to merge clouds: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
