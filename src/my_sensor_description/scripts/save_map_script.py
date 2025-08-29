#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
import os
import sys

class ManualMapSaver(Node):
    def __init__(self):
        super().__init__('manual_map_saver')

        # Declare and get the output path parameter
        self.declare_parameter('output_path', '~/output.bt')
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.output_path = os.path.expanduser(self.output_path) # Expands '~' to /home/user

        self.get_logger().info(f"Waiting for one message on /octomap_binary to save to {self.output_path}")

        # Create a subscription. The callback will be triggered once.
        self.subscription = self.create_subscription(
            Octomap,
            '/octomap_binary',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('OctoMap message received!')

        try:
            # Ensure the directory exists
            output_dir = os.path.dirname(self.output_path)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)

            # Write the binary data from the message directly to the file
            with open(self.output_path, 'wb') as f:
                f.write(bytes(msg.data))

            self.get_logger().info(f"Successfully saved map to {self.output_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")

        # Shutdown the node after saving the file
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    map_saver_node = ManualMapSaver()
    rclpy.spin(map_saver_node)

if __name__ == '__main__':
    main()