#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.time import Time


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('point_cloud_merger')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.topic_list = [
            '/camera_robot/camera_robot_sensor/points',
            '/camera_robot_1/camera_robot_sensor_1/points',
            '/lidar_robot/lidar_robot_scan/out'
        ]

        self.target_frame = 'map'
        self.clouds = {}
        self._subs = []

        for topic in self.topic_list:
            self.clouds[topic] = None
            sub = self.create_subscription(PointCloud2, topic, self.make_callback(topic), 10)
            self._subs.append(sub)

        self.pub = self.create_publisher(PointCloud2, '/mergen_sensor_points', 10)
        self.create_timer(1.0, self.merge_and_publish)  # 1 Hz publishing

    def make_callback(self, topic_name):
        def callback(msg):
            try:
                # Lookup transform at the time of the message
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )

                # Strip to only x, y, z for OctoMap compatibility
                sanitized_cloud = self.strip_to_xyz(msg)

                # Transform to 'map' frame
                transformed_cloud = do_transform_cloud(sanitized_cloud, transform)
                transformed_cloud.header.stamp = self.get_clock().now().to_msg()
                transformed_cloud.header.frame_id = self.target_frame

                self.clouds[topic_name] = transformed_cloud
            except Exception as e:
                self.get_logger().warn(f'[{topic_name}] Failed to transform: {e}')
        return callback

    def strip_to_xyz(self, cloud):
        """Remove all fields except x, y, z for OctoMap."""
        points = list(pc2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True))
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud(cloud.header, fields, points)

    def merge_and_publish(self):
        if not all(self.clouds.values()):
            self.get_logger().info('Waiting for all clouds...')
            return

        try:
            all_points = []
            for cloud in self.clouds.values():
                all_points.extend(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))

            header = next(iter(self.clouds.values())).header
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.target_frame

            merged_cloud = pc2.create_cloud_xyz32(header, all_points)

            self.pub.publish(merged_cloud)
            self.get_logger().info(f'Published merged cloud with {len(all_points)} points')
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
