#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import open3d as o3d
import numpy as np
import struct

from sensor_msgs.msg import PointField

class VoxelFilterNode(Node):
    def __init__(self):
        super().__init__('voxel_filter_node')
        self.declare_parameter('leaf_size', 0.2)

        self.leaf_size = self.get_parameter('leaf_size').value

        # Subscriptions
        self.sub1 = self.create_subscription(
            PointCloud2,
            '/camera_robot/camera_robot_sensor/points',
            self.callback_cam1,
            10
        )
        self.sub2 = self.create_subscription(
            PointCloud2,
            '/camera_robot_1/camera_robot_sensor_1/points',
            self.callback_cam2,
            10
        )

        # Publishers
        self.pub1 = self.create_publisher(PointCloud2, '/camera_robot/camera_robot_sensor/downsampled', 10)
        self.pub2 = self.create_publisher(PointCloud2, '/camera_robot_1/camera_robot_sensor_1/downsampled', 10)

        self.get_logger().info(f'VoxelFilterNode initialized with leaf size: {self.leaf_size}')

    def callback_cam1(self, msg):
        filtered = self.downsample_pointcloud(msg)
        if filtered:
            self.pub1.publish(filtered)

    def callback_cam2(self, msg):
        filtered = self.downsample_pointcloud(msg)
        if filtered:
            self.pub2.publish(filtered)

    def downsample_pointcloud(self, msg):
        try:
            # Convert PointCloud2 to numpy
            points = np.array([
                [x, y, z, rgb] for x, y, z, rgb in point_cloud2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
            ], dtype=np.float32)

            if len(points) == 0:
                return None

            # Convert RGB float to 3-channel uint8
            xyz = points[:, :3]
            rgb_floats = points[:, 3]
            rgb_uint8 = np.array([self.unpack_rgb_float(f) for f in rgb_floats], dtype=np.uint8)
            colors = rgb_uint8 / 255.0

            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            # Apply voxel grid filter
            down_pcd = pcd.voxel_down_sample(voxel_size=self.leaf_size)

            if len(down_pcd.points) == 0:
                return None

            # Convert back to numpy with RGB packed as float
            xyz = np.asarray(down_pcd.points)
            rgb = np.asarray(down_pcd.colors) * 255.0
            rgb_packed = [self.pack_rgb_uint8(*c.astype(np.uint8)) for c in rgb]

            final_points = np.column_stack((xyz, rgb_packed))

            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            return point_cloud2.create_cloud(header, fields, final_points.tolist())

        except Exception as e:
            self.get_logger().error(f'Error during downsampling: {str(e)}')
            return None

    def unpack_rgb_float(self, rgb_float):
        rgb_int = int(rgb_float)
        r = (rgb_int >> 16) & 255
        g = (rgb_int >> 8) & 255
        b = rgb_int & 255
        return [r, g, b]

    def pack_rgb_uint8(self, r, g, b):
        rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
        return struct.unpack('f', struct.pack('I', rgb_int))[0]

def main(args=None):
    rclpy.init(args=args)
    node = VoxelFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
