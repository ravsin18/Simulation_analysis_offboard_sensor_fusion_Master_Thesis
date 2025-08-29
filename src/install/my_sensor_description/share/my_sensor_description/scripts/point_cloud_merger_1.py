#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
import struct
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('point_cloud_merger')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cloud1 = None
        self.cloud2 = None

        self.target_frame = 'map'  # Set this to 'odom' or 'base_link' if preferred

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

    def sanitize_xyz(self, cloud):
        """
        Return a brand-new, pure-[x,y,z] PointCloud2:
        - fields = [x,y,z] FLOAT32, packed consecutively
        - data   = struct.pack('fff',x,y,z) for each point
        """
        header = cloud.header
        # read only x,y,z
        xyz_iter = pc2.read_points(cloud, field_names=['x','y','z'], skip_nans=True)

        # build a minimal FLOAT32 XYZ cloud
        new_cloud = PointCloud2()
        new_cloud.header = header
        new_cloud.height = 1
        points = list(xyz_iter)
        new_cloud.width = len(points)
        new_cloud.is_dense = cloud.is_dense
        new_cloud.is_bigendian = cloud.is_bigendian

        # exactly three FLOAT32 fields, consecutively packed
        new_cloud.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        new_cloud.point_step = 12
        new_cloud.row_step = new_cloud.point_step * new_cloud.width

        # pack the data
        buf = bytearray(new_cloud.row_step)
        for i, (x, y, z) in enumerate(points):
            struct.pack_into('fff', buf, i * 12, x, y, z)
        new_cloud.data = bytes(buf)
        return new_cloud

    def strip_to_xyz(self, cloud):
        # Convert to xyz-only cloud (removes rgb/intensity/etc.)
        points = list(pc2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True))
        return pc2.create_cloud_xyz32(cloud.header, points)

    def callback1(self, msg):
        try:
            stripped = self.strip_to_xyz(msg)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, rclpy.time.Time()
            )
            self.cloud1 = self.sanitize_xyz(do_transform_cloud(stripped, transform))
            self.merge_and_publish()
        except Exception as e:
            self.get_logger().warn(f'Failed to transform cloud1: {e}')

    def callback2(self, msg):
        try:
            stripped = self.strip_to_xyz(msg)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
            self.cloud2 = self.sanitize_xyz(do_transform_cloud(stripped, transform))
            self.merge_and_publish()
        except Exception as e:
            self.get_logger().warn(f'Failed to transform cloud2: {e}')

    def merge_and_publish(self):
        if self.cloud1 is None or self.cloud2 is None:
            return

        try:
            points1 = list(pc2.read_points(self.cloud1, field_names=("x", "y", "z"), skip_nans=True))
            points2 = list(pc2.read_points(self.cloud2, field_names=("x", "y", "z"), skip_nans=True))

            merged_points =points1+points2
            merged_cloud = pc2.create_cloud_xyz32(self.cloud1.header, merged_points)

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
