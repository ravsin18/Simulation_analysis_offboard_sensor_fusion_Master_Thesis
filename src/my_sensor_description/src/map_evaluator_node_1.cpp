#include <cstddef> // <-- THE FIX: Add this line at the very top
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <fstream> // For file I/O
#include <rclcpp/rclcpp.hpp>

// ROS Messages
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// OctoMap
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

// Custom Service Header from your interfaces package
#include "my_sensor_description_interfaces/srv/start_timer.hpp"

// Enum to represent the final projected state of a 2D grid cell
enum class CellClassification {
    TRUE_NEGATIVE = 0,
    FALSE_POSITIVE = 90,
    FALSE_NEGATIVE = 95,
    TRUE_POSITIVE = 100,
    UNKNOWN = -1
};

class AdvancedOctomapEvaluator : public rclcpp::Node {
public:
    AdvancedOctomapEvaluator() : Node("advanced_octomap_evaluator") {
        using std::placeholders::_1;
        using std::placeholders::_2;

        RCLCPP_INFO(this->get_logger(), "Advanced Octomap Evaluator Node started.");

        // --- Parameters ---
        this->declare_parameter<std::string>("ground_truth_filename", "");
        this->declare_parameter<double>("eval_min_x", -10.0);
        this->declare_parameter<double>("eval_min_y", -4.25);
        this->declare_parameter<double>("eval_min_z", 0.05);
        this->declare_parameter<double>("eval_max_x", 6.0);
        this->declare_parameter<double>("eval_max_y", 4.5);
        this->declare_parameter<double>("eval_max_z", 2.0);
        this->declare_parameter<std::string>("csv_output_path", "/home/singh/ros2_sensor_ws/evaluation/with_noise/warehouse/warehouse_C_noise.csv");

        // --- Load Ground Truth Map ---
        std::string gt_filename = this->get_parameter("ground_truth_filename").as_string();
        if (gt_filename.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Required parameter 'ground_truth_filename' not set.");
            rclcpp::shutdown();
            return;
        }
        gt_octree_ = std::make_shared<octomap::OcTree>(0.1);
        if (!gt_octree_->readBinary(gt_filename)) {
            RCLCPP_FATAL(this->get_logger(), "Could not read ground truth file: %s", gt_filename.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully loaded ground truth map from: %s", gt_filename.c_str());

        // --- Publishers ---
        projected_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/evaluation_projected_map", rclcpp::QoS(1).transient_local());
        // Comparison Markers
        marker_tp_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers/true_positives", 10);
        marker_fp_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers/false_positives", 10);
        marker_fn_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers/false_negatives", 10);
        // General Status Markers
        marker_occ_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers/occupied", 10);
        marker_free_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers/free", 10);
        marker_unknown_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_markers/unknown", 10);
        
        // --- Subscriber ---
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_custom", rclcpp::SystemDefaultsQoS(), std::bind(&AdvancedOctomapEvaluator::octomap_callback, this, _1));

        // --- Service Server ---
        start_timer_service_ = this->create_service<my_sensor_description_interfaces::srv::StartTimer>(
            "/start_evaluation_timer", std::bind(&AdvancedOctomapEvaluator::start_timer_callback, this, _1, _2));
    }

private:
    // ROS Communications
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr projected_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_tp_pub_, marker_fp_pub_, marker_fn_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_occ_pub_, marker_free_pub_, marker_unknown_pub_;
    rclcpp::Service<my_sensor_description_interfaces::srv::StartTimer>::SharedPtr start_timer_service_;
    rclcpp::TimerBase::SharedPtr evaluation_timer_;

    // Data
    std::shared_ptr<octomap::OcTree> gt_octree_;
    std::string csv_output_path_;

    // Member variables for CSV export
    rclcpp::Time last_evaluation_time_;
    double last_completeness_ = 0.0;
    long last_occupied_count_ = 0, last_free_count_ = 0, last_unknown_count_ = 0;
    long last_tp_count_ = 0, last_fp_count_ = 0, last_tn_count_ = 0, last_fn_count_ = 0;
    double last_precision_ = 0.0, last_recall_ = 0.0, last_f1_score_ = 0.0, last_iou_ = 0.0;
    double last_memory_mb_ = 0.0;
    double last_callback_duration_ms_ = 0.0;

    // Service callback
    void start_timer_callback(
        const std::shared_ptr<my_sensor_description_interfaces::srv::StartTimer::Request> request,
        std::shared_ptr<my_sensor_description_interfaces::srv::StartTimer::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to generate CSV report in %.2f seconds.", request->duration_seconds);
        if (evaluation_timer_) { evaluation_timer_->cancel(); }
        evaluation_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(request->duration_seconds),
            std::bind(&AdvancedOctomapEvaluator::generate_csv_report, this));
        response->success = true;
        response->message = "Timer started. CSV will be generated upon completion.";
    }

    // CSV generation function
    void generate_csv_report() {
        RCLCPP_INFO(this->get_logger(), "Timer expired. Generating CSV report...");
        csv_output_path_ = this->get_parameter("csv_output_path").as_string();
        std::ofstream csv_file(csv_output_path_);
        if (!csv_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file at: %s", csv_output_path_.c_str());
            return;
        }

        csv_file << "Metric,Value\n";
        csv_file << "Evaluation Time (s)," << last_evaluation_time_.seconds() << "\n";
        csv_file << "Completeness (%)," << last_completeness_ * 100.0 << "\n";
        csv_file << "Occupied Voxels," << last_occupied_count_ << "\n";
        csv_file << "Free Voxels," << last_free_count_ << "\n";
        csv_file << "Unknown Voxels," << last_unknown_count_ << "\n";
        csv_file << "3D True Positives," << last_tp_count_ << "\n";
        csv_file << "3D False Positives," << last_fp_count_ << "\n";
        csv_file << "3D True Negatives," << last_tn_count_ << "\n";
        csv_file << "3D False Negatives," << last_fn_count_ << "\n";
        csv_file << "3D Precision," << last_precision_ << "\n";
        csv_file << "3D Recall," << last_recall_ << "\n";
        csv_file << "3D F1-Score," << last_f1_score_ << "\n";
        csv_file << "3D IoU," << last_iou_ << "\n";
        csv_file << "Memory Usage (MB)," << last_memory_mb_ << "\n";
        csv_file << "Callback Time (ms)," << last_callback_duration_ms_ << "\n";
        
        csv_file.close();
        RCLCPP_INFO(this->get_logger(), "Successfully wrote evaluation metrics to %s", csv_output_path_.c_str());
        if (evaluation_timer_) { evaluation_timer_->cancel(); }
    }

    // Main evaluation callback
    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        auto start_time = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Received new map, starting full evaluation...");

        auto generated_tree = std::unique_ptr<octomap::AbstractOcTree>(octomap_msgs::binaryMsgToMap(*msg));
        auto generated_octree = dynamic_cast<octomap::OcTree*>(generated_tree.get());
        if (!generated_octree) { return; }

        double min_x = this->get_parameter("eval_min_x").as_double();
        double min_y = this->get_parameter("eval_min_y").as_double();
        double min_z = this->get_parameter("eval_min_z").as_double();
        double max_x = this->get_parameter("eval_max_x").as_double();
        double max_y = this->get_parameter("eval_max_y").as_double();
        double max_z = this->get_parameter("eval_max_z").as_double();
        double resolution = generated_octree->getResolution();
        
        long occupied_count = 0, free_count = 0, unknown_count = 0;
        long tp_count = 0, fp_count = 0, tn_count = 0, fn_count = 0;
        
        visualization_msgs::msg::MarkerArray markers_tp, markers_fp, markers_fn;
        visualization_msgs::msg::MarkerArray markers_occ, markers_free, markers_unknown;
        int id_counter = 0;
        
        nav_msgs::msg::OccupancyGrid projected_map;
        projected_map.header.frame_id = "map";
        projected_map.info.resolution = resolution;
        projected_map.info.width = static_cast<unsigned int>((max_x - min_x) / resolution);
        projected_map.info.height = static_cast<unsigned int>((max_y - min_y) / resolution);
        projected_map.info.origin.position.x = min_x;
        projected_map.info.origin.position.y = min_y;
        projected_map.info.origin.position.z = 0.0;
        projected_map.data.assign(projected_map.info.width * projected_map.info.height, static_cast<int8_t>(CellClassification::UNKNOWN));

        // --- Main Evaluation Loop ---
        for (float x = min_x; x < max_x; x += resolution) {
            for (float y = min_y; y < max_y; y += resolution) {
                CellClassification column_result = CellClassification::UNKNOWN;
                for (float z = min_z; z < max_z; z += resolution) {
                    octomap::point3d point(x, y, z);
                    
                    octomap::OcTreeNode* gen_node = generated_octree->search(point);
                    bool is_gen_occupied = gen_node && generated_octree->isNodeOccupied(gen_node);
                    bool is_gen_free = gen_node && !generated_octree->isNodeOccupied(gen_node);

                    octomap::OcTreeNode* gt_node = gt_octree_->search(point);
                    bool is_gt_occupied = gt_node && gt_octree_->isNodeOccupied(gt_node);
                    
                    if (is_gen_occupied) {
                        occupied_count++;
                        markers_occ.markers.push_back(createVoxelMarker(point, resolution, id_counter, "occupied"));
                        if (is_gt_occupied) { 
                            tp_count++;
                            column_result = std::max(column_result, CellClassification::TRUE_POSITIVE);
                            markers_tp.markers.push_back(createVoxelMarker(point, resolution, id_counter, "true_positive"));
                        } else { 
                            fp_count++;
                            column_result = std::max(column_result, CellClassification::FALSE_POSITIVE);
                            markers_fp.markers.push_back(createVoxelMarker(point, resolution, id_counter, "false_positive"));
                        }
                    } else if (is_gen_free) {
                        free_count++;
                        markers_free.markers.push_back(createVoxelMarker(point, resolution, id_counter, "free"));
                        if (is_gt_occupied) { 
                            fn_count++;
                            column_result = std::max(column_result, CellClassification::FALSE_NEGATIVE);
                            markers_fn.markers.push_back(createVoxelMarker(point, resolution, id_counter, "false_negative"));
                        } else { 
                            tn_count++;
                            column_result = std::max(column_result, CellClassification::TRUE_NEGATIVE);
                        }
                    } else {
                        unknown_count++;
                        markers_unknown.markers.push_back(createVoxelMarker(point, resolution, id_counter, "unknown"));
                    }
                    id_counter++;
                }
                int map_x = static_cast<int>((x - min_x) / resolution);
                int map_y = static_cast<int>((y - min_y) / resolution);
                int index = map_x + map_y * projected_map.info.width;
                if (index >= 0 && index < projected_map.data.size()) {
                    projected_map.data[index] = static_cast<int8_t>(column_result);
                }
            }
        }
        
        // --- Publish all visualizations ---
        projected_map.header.stamp = this->now();
        projected_map_pub_->publish(projected_map);
        marker_tp_pub_->publish(markers_tp);
        marker_fp_pub_->publish(markers_fp);
        marker_fn_pub_->publish(markers_fn);
        marker_occ_pub_->publish(markers_occ);
        marker_free_pub_->publish(markers_free);
        marker_unknown_pub_->publish(markers_unknown);
        
        // --- Finalize and Calculate All Metrics ---
        long total_voxels = occupied_count + free_count + unknown_count;
        double completeness = (total_voxels > 0) ? static_cast<double>(occupied_count + free_count) / total_voxels : 0.0;
        
        double precision = 0.0, recall = 0.0, f1_score = 0.0, iou = 0.0;
        if (tp_count + fp_count > 0) precision = static_cast<double>(tp_count) / (tp_count + fp_count);
        if (tp_count + fn_count > 0) recall = static_cast<double>(tp_count) / (tp_count + fn_count);
        if (precision + recall > 0) f1_score = 2.0 * (precision * recall) / (precision + recall);
        if (tp_count + fp_count + fn_count > 0) iou = static_cast<double>(tp_count) / (tp_count + fp_count + fn_count);

        double memory_mb = generated_octree->memoryUsage() / 1024.0 / 1024.0;
        auto end_time = this->get_clock()->now();
        double callback_duration_ms = (end_time - start_time).seconds() * 1000.0;

        // --- Print all metrics to the terminal logger ---
        RCLCPP_INFO(this->get_logger(), "--- MAP EVALUATION (Sim Time: %.2fs) ---", this->now().seconds());
        RCLCPP_INFO(this->get_logger(), "[Map Quality]");
        RCLCPP_INFO(this->get_logger(), "  - Completeness: %.2f %%", completeness * 100.0);
        RCLCPP_INFO(this->get_logger(), "  - Voxel Counts (Occ/Free/Unknown): %ld / %ld / %ld", occupied_count, free_count, unknown_count);
        RCLCPP_INFO(this->get_logger(), "[Map Accuracy vs. Ground Truth]");
        RCLCPP_INFO(this->get_logger(), "  - 3D Voxel Counts (TP/FP/TN/FN): %ld / %ld / %ld / %ld", tp_count, fp_count, tn_count, fn_count);
        RCLCPP_INFO(this->get_logger(), "  - 3D Precision: %.4f | 3D Recall: %.4f | 3D F1-Score: %.4f", precision, recall, f1_score);
        RCLCPP_INFO(this->get_logger(), "  - 3D Intersection over Union (IoU): %.4f", iou);
        RCLCPP_INFO(this->get_logger(), "[Algorithm Efficiency]");
        RCLCPP_INFO(this->get_logger(), "  - Memory Usage: %.4f MB", memory_mb);
        RCLCPP_INFO(this->get_logger(), "  - Evaluation Callback Time: %.2f ms", callback_duration_ms);
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");

        // --- Store all metrics in member variables for potential CSV export ---
        last_evaluation_time_ = this->now();
        last_completeness_ = completeness;
        last_occupied_count_ = occupied_count;
        last_free_count_ = free_count;
        last_unknown_count_ = unknown_count;
        last_tp_count_ = tp_count;
        last_fp_count_ = fp_count;
        last_tn_count_ = tn_count;
        last_fn_count_ = fn_count;
        last_precision_ = precision;
        last_recall_ = recall;
        last_f1_score_ = f1_score;
        last_iou_ = iou;
        last_memory_mb_ = memory_mb;
        last_callback_duration_ms_ = callback_duration_ms;
    }

    // Helper function to create a base marker for visualization
    visualization_msgs::msg::Marker createVoxelMarker(const octomap::point3d& point, double size, int id, const std::string& type) {
        visualization_msgs::msg::Marker cube;
        cube.header.frame_id = "map";
        cube.header.stamp = this->now();
        cube.ns = type;
        cube.id = id;
        cube.type = visualization_msgs::msg::Marker::CUBE;
        cube.action = visualization_msgs::msg::Marker::ADD;
        cube.pose.position.x = point.x();
        cube.pose.position.y = point.y();
        cube.pose.position.z = point.z();
        cube.scale.x = cube.scale.y = cube.scale.z = size;
        cube.lifetime = rclcpp::Duration::from_seconds(10.0);

        // Comparison colors
        if (type == "true_positive") { cube.color.r = 0.0f; cube.color.g = 1.0f; cube.color.b = 0.0f; cube.color.a = 0.7f; } // Green
        else if (type == "false_positive") { cube.color.r = 1.0f; cube.color.g = 0.0f; cube.color.b = 0.0f; cube.color.a = 0.7f; } // Red
        else if (type == "false_negative") { cube.color.r = 1.0f; cube.color.g = 0.5f; cube.color.b = 0.0f; cube.color.a = 0.7f; } // Orange
        // General status colors
        else if (type == "occupied") { cube.color.r = 0.6f; cube.color.g = 0.0f; cube.color.b = 1.0f; cube.color.a = 0.5f; } // Purple
        else if (type == "free") { cube.color.r = 0.0f; cube.color.g = 1.0f; cube.color.b = 1.0f; cube.color.a = 0.05f; } // Transparent Cyan
        else if (type == "unknown") { cube.color.r = 1.0f; cube.color.g = 1.0f; cube.color.b = 1.0f; cube.color.a = 0.1f; } // Transparent White
        
        return cube;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdvancedOctomapEvaluator>());
    rclcpp::shutdown();
    return 0;
}