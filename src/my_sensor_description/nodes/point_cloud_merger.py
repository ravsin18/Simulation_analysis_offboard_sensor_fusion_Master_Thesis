import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('point_cloud_merger')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cloud1 = None
        self.cloud2 = None

        self.target_frame = 'map'  # Change to 'odom' or 'base_link' if needed

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
            self.cloud1 = self.tf_buffer.transform(msg, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.5))
            self.merge_and_publish()
        except Exception as e:
            self.get_logger().warn(f'Failed to transform cloud1: {e}')

    def callback2(self, msg):
        try:
            self.cloud2 = self.tf_buffer.transform(msg, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.5))
            self.merge_and_publish()
        except Exception as e:
            self.get_logger().warn(f'Failed to transform cloud2: {e}')

    def merge_and_publish(self):
        if self.cloud1 is None or self.cloud2 is None:
            return

        points1 = list(pc2.read_points(self.cloud1, field_names=("x", "y", "z"), skip_nans=True))
        points2 = list(pc2.read_points(self.cloud2, field_names=("x", "y", "z"), skip_nans=True))

        merged_points = points1 + points2
        header = self.cloud1.header
        merged_cloud = pc2.create_cloud_xyz32(header, merged_points)

        self.pub.publish(merged_cloud)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
