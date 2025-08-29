#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <octomap/OcTree.h>

class OctomapEvaluator : public rclcpp::Node {
public:
  OctomapEvaluator()
      : Node("octomap_evaluator") {
    using std::placeholders::_1;
    RCLCPP_INFO(this->get_logger(), "Octomap Evaluator Node started.");

    marker_all_pub_      = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers", 10);
    marker_occ_pub_      = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers/occupied", 10);
    marker_free_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers/free", 10);
    marker_unknown_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers/unknown", 10);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_custom", 10, std::bind(&OctomapEvaluator::octomap_callback, this, _1));
  }

private:
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_all_pub_, marker_occ_pub_, marker_free_pub_, marker_unknown_pub_;

  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::binaryMsgToMap(*msg));
    if (!tree) {
      RCLCPP_ERROR(this->get_logger(), "Failed to deserialize OctoMap");
      return;
    }

    auto octree = dynamic_cast<octomap::OcTree*>(tree.get());
    if (!octree) {
      RCLCPP_ERROR(this->get_logger(), "Failed to cast to OcTree");
      return;
    }

    float min_x = -10, max_x = 6.0;
    float min_y = -4.25, max_y = 4.5;
    float min_z = 0.0, max_z = 2.0;

    int occupied_count = 0, free_count = 0, unknown_count = 0;
    double conf_occ = 0.0, conf_free = 0.0;

    visualization_msgs::msg::MarkerArray markers_all, markers_occ, markers_free, markers_unknown;
    int id = 0;

    for (float x = min_x; x < max_x; x += octree->getResolution()) {
      for (float y = min_y; y < max_y; y += octree->getResolution()) {
        for (float z = min_z; z < max_z; z += octree->getResolution()) {
          octomap::OcTreeNode* node = octree->search(x, y, z);

          visualization_msgs::msg::Marker cube;
          cube.header.frame_id = "map";
          cube.header.stamp = this->now();
          cube.ns = "voxels";
          cube.id = id++;
          cube.type = visualization_msgs::msg::Marker::CUBE;
          cube.action = visualization_msgs::msg::Marker::ADD;
          cube.pose.position.x = x;
          cube.pose.position.y = y;
          cube.pose.position.z = z;
          cube.scale.x = cube.scale.y = cube.scale.z = octree->getResolution();
          cube.lifetime = rclcpp::Duration::from_seconds(0.0);

          if (node) {
            float prob = node->getOccupancy();
            if (octree->isNodeOccupied(node)) {
              occupied_count++;
              conf_occ += prob;
              cube.color.r = 1.0; cube.color.g = 0.0; cube.color.b = 0.0; cube.color.a = 0.6;
              markers_occ.markers.push_back(cube);
            } else {
              free_count++;
              conf_free += prob;
              cube.color.r = 0.0; cube.color.g = 1.0; cube.color.b = 0.0; cube.color.a = 0.3;
              markers_free.markers.push_back(cube);
            }
          } else {
            unknown_count++;
            cube.color.r = 0.5; cube.color.g = 0.5; cube.color.b = 0.5; cube.color.a = 1;
            markers_unknown.markers.push_back(cube);
          }

          markers_all.markers.push_back(cube);
        }
      }
    }

    // Publish all marker arrays
    marker_all_pub_->publish(markers_all);
    marker_occ_pub_->publish(markers_occ);
    marker_free_pub_->publish(markers_free);
    marker_unknown_pub_->publish(markers_unknown);

    int total = occupied_count + free_count + unknown_count;
    double avg_occ_conf = (occupied_count > 0) ? conf_occ / occupied_count : 0.0;
    double avg_free_conf = (free_count > 0) ? conf_free / free_count : 0.0;
    double avg_total_conf = (occupied_count + free_count > 0) ? (conf_occ + conf_free) / (occupied_count + free_count) : 0.0;
    double completeness = (total > 0) ? static_cast<double>(occupied_count + free_count) / total : 0.0;

    RCLCPP_INFO(this->get_logger(), "Octomap Evaluation:");
    RCLCPP_INFO(this->get_logger(), "  Occupied Voxels         : %d", occupied_count);
    RCLCPP_INFO(this->get_logger(), "  Free Voxels             : %d", free_count);
    RCLCPP_INFO(this->get_logger(), "  Unknown Voxels          : %d", unknown_count);
    RCLCPP_INFO(this->get_logger(), "  Avg. Confidence (Occ)   : %.4f", avg_occ_conf);
    RCLCPP_INFO(this->get_logger(), "  Avg. Confidence (Free)  : %.4f", avg_free_conf);
    RCLCPP_INFO(this->get_logger(), "  Avg. Confidence (Total) : %.4f", avg_total_conf);
    RCLCPP_INFO(this->get_logger(), "  Map Completeness        : %.2f%%", completeness * 100.0);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapEvaluator>());
  rclcpp::shutdown();
  return 0;
}
