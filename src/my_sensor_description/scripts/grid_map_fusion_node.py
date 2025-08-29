#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np


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

        self.pub = self.create_publisher(PointCloud2, '/merged_sensor_points', 10)
        self.create_timer(1.0, self.merge_and_publish)  # 1 Hz publishing

    def make_callback(self, topic_name):
        def callback(msg):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )

                sanitized_cloud = self.strip_to_xyz(msg)
                transformed_cloud = do_transform_cloud(sanitized_cloud, transform)
                transformed_cloud.header.stamp = self.get_clock().now().to_msg()
                transformed_cloud.header.frame_id = self.target_frame

                self.clouds[topic_name] = transformed_cloud
            except Exception as e:
                self.get_logger().warn(f"[{topic_name}] Failed to transform: {e}")
        return callback

    def strip_to_xyz(self, cloud):
        """Sanitize the cloud to contain only x/y/z float32 points and skip NaN/inf."""
        points = []
        for pt in pc2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True):
            if all(np.isfinite(pt)) and len(pt) == 3:
                points.append((float(pt[0]), float(pt[1]), float(pt[2])))

        # Limit number of points for performance
        MAX_POINTS = 50000
        if len(points) > MAX_POINTS:
            self.get_logger().warn(f"Clipping input cloud to {MAX_POINTS} points")
            points = points[:MAX_POINTS]

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
                for pt in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
                    if all(np.isfinite(pt)) and len(pt) == 3:
                        all_points.append((float(pt[0]), float(pt[1]), float(pt[2])))

            # Safety limit on total points
            MAX_TOTAL_POINTS = 80000
            if len(all_points) > MAX_TOTAL_POINTS:
                self.get_logger().warn(f"Merged cloud trimmed to {MAX_TOTAL_POINTS} points")
                all_points = all_points[:MAX_TOTAL_POINTS]

            header = next(iter(self.clouds.values())).header
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.target_frame

            merged_cloud = pc2.create_cloud_xyz32(header, all_points)
            merged_cloud.is_dense = True  # Required for OctoMap

            self.pub.publish(merged_cloud)
            self.get_logger().info(f"Published merged cloud with {len(all_points)} points (Dense: True)")

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
