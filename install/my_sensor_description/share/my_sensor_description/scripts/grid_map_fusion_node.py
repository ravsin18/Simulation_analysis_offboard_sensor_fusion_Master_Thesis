import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2

class GridMapFusionNode(Node):
    def __init__(self):
        super().__init__('grid_map_fusion_node')

        # Map parameters
        self.map_size = 10.0  # meters
        self.resolution = 0.1  # meters per cell
        self.grid_cells = int(self.map_size / self.resolution)
        self.origin = -self.map_size / 2  # center the map

        # Log-odds parameters
        self.log_odds_occ = np.log(0.7 / 0.3)
        self.log_odds_free = np.log(0.3 / 0.7)
        self.log_odds_max = np.log(0.97 / 0.03)
        self.log_odds_min = np.log(0.03 / 0.97)

        # Initialize grid
        self.log_odds_grid = np.zeros((self.grid_cells, self.grid_cells))

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(PointCloud2, '/lidar_robot/lidar_robot_scan/out', self.pointcloud_callback, 10)
        self.create_subscription(PointCloud2, '/camera_robot/camera_robot_sensor/points', self.pointcloud_callback, 10)
        self.create_subscription(PointCloud2, '/camera_robot_1/camera_robot_sensor_1/points', self.pointcloud_callback, 10)

        # Publisher
        self.grid_pub = self.create_publisher(GridMap, '/fused_occupancy_map', 10)

    def pointcloud_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
            cloud_transformed = tf2_sensor_msgs.do_transform_cloud(msg, transform)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        points = pc2.read_points(cloud_transformed, field_names=("x", "y", "z"), skip_nans=True)

        for p in points:
            x, y, z = p
            if z < 0.2 or z > 3.0:
                continue
            i, j = self.xy_to_grid(x, y)
            if not self.valid_cell(i, j):
                continue
            self.log_odds_grid[i, j] += self.log_odds_occ
            self.log_odds_grid[i, j] = np.clip(self.log_odds_grid[i, j], self.log_odds_min, self.log_odds_max)

        self.publish_grid()

    def xy_to_grid(self, x, y):
        i = int((x - self.origin) / self.resolution)
        j = int((y - self.origin) / self.resolution)
        return i, j

    def valid_cell(self, i, j):
        return 0 <= i < self.grid_cells and 0 <= j < self.grid_cells

    def publish_grid(self):
        grid_msg = GridMap()
        grid_msg.info.resolution = self.resolution
        grid_msg.info.length_x = self.map_size
        grid_msg.info.length_y = self.map_size
        grid_msg.info.pose.position.x = 0.0
        grid_msg.info.pose.position.y = 0.0
        grid_msg.info.pose.position.z = 0.0
        grid_msg.info.pose.orientation.w = 1.0

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        grid_msg.header = header

        prob_grid = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds_grid))
        grid_msg.layers.append('occupancy')
        grid_msg.data.append(prob_grid.flatten().tolist())

        self.grid_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GridMapFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
