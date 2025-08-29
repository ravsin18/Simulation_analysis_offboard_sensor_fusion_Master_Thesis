#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import os

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')

        # Declare and get the 'pcd_file' parameter
        self.declare_parameter('pcd_file', '')
        pcd_file = self.get_parameter('pcd_file').get_parameter_value().string_value

        if not os.path.exists(pcd_file):
            self.get_logger().error(f"PCD file does not exist at: {pcd_file}")
            raise FileNotFoundError(f"PCD file not found: {pcd_file}")

        self.publisher_ = self.create_publisher(PointCloud2, 'map_pcd', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load PCD file
        self.pcd = o3d.io.read_point_cloud(pcd_file)
        self.points = np.asarray(self.pcd.points)
        self.get_logger().info(f"Loaded {len(self.points)} points from {pcd_file}")

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        cloud_msg = pc2.create_cloud_xyz32(header, self.points)
        self.publisher_.publish(cloud_msg)
        self.get_logger().info('Publishing PCD point cloud')

def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'map_pcd', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load PCD file
        self.pcd = o3d.io.read_point_cloud('/home/singh/ros2_sensor_ws/map.pcd')
        self.points = np.asarray(self.pcd.points)
        print(f"Loaded {len(self.points)} points from map.pcd")


    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        cloud_msg = pc2.create_cloud_xyz32(header, self.points)
        self.publisher_.publish(cloud_msg)
        self.get_logger().info('Publishing map.pcd')

def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''
