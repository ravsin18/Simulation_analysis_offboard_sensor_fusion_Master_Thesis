#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

// ADDED: Headers for PCL and PCL-ROS conversions
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <string>
#include <memory>
#include <mutex> // ADDED: For thread safety with the merged cloud

class OctomapRaycastingNode : public rclcpp::Node
{
public:
    OctomapRaycastingNode()
    : Node("octomap_raycasting_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Parameters
        this->declare_parameter<std::vector<std::string>>("input_topics", std::vector<std::string>());
        this->declare_parameter<std::string>("target_frame", "map");
        this->declare_parameter<double>("resolution", 0.1);

        this->get_parameter("input_topics", input_topics_);
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("resolution", resolution_);

        octree_ = std::make_shared<octomap::OcTree>(resolution_);

        // ADDED: Initialize the PCL point cloud accumulator
        merged_cloud_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        // Subscriptions
        for (const auto &topic : input_topics_) {
            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, rclcpp::SystemDefaultsQoS(), // Using SystemDefaultsQoS for better compatibility
                [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    this->processPointCloud(msg, topic);
                }
            );
            subscriptions_.push_back(sub);
        }

        // Publishers
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/octomap_custom", 10);
        // ADDED: Create the publisher for the merged point cloud
        merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_point_cloud", 10);


        // MODIFIED: Timer to publish all data periodically
        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&OctomapRaycastingNode::publishMaps, this) // Renamed function
        );

        RCLCPP_INFO(this->get_logger(), "Octomap Raycasting Node initialized.");
    }

private:
    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &topic)
    {
        geometry_msgs::msg::TransformStamped transform;

        try {
            transform = tf_buffer_.lookupTransform(
                target_frame_, msg->header.frame_id,
                tf2::TimePointZero
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "[%s] Transform error: %s", topic.c_str(), ex.what());
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud_transformed;
        try {
            tf2::doTransform(*msg, cloud_transformed, transform);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "[%s] PointCloud transform failed: %s", topic.c_str(), ex.what());
            return;
        }

        // --- Start of OctoMap Update Logic (Unchanged) ---
        double sensor_x = transform.transform.translation.x;
        double sensor_y = transform.transform.translation.y;
        double sensor_z = transform.transform.translation.z;

        size_t inserted = 0;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_transformed, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_transformed, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_transformed, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            octomap::point3d origin(sensor_x, sensor_y, sensor_z);
            octomap::point3d endpoint(*iter_x, *iter_y, *iter_z);
            if ((origin - endpoint).norm() < 0.01) continue; 

            octree_->insertRay(origin, endpoint);
            inserted++;
        }
        // --- End of OctoMap Update Logic ---

        // ADDED: Logic to merge the transformed point cloud
        {
            pcl::PointCloud<pcl::PointXYZ> incoming_cloud_pcl;
            pcl::fromROSMsg(cloud_transformed, incoming_cloud_pcl);

            std::lock_guard<std::mutex> lock(cloud_mutex_);
            *merged_cloud_pcl_ += incoming_cloud_pcl;
        }

        RCLCPP_INFO(this->get_logger(), "Inserted %zu rays from topic [%s]", inserted, topic.c_str());
    }

    // MODIFIED: Renamed function to reflect its new purpose
    // MODIFIED: Replaced function with a more verbose version for debugging and robust publishing
    void publishMaps()
    {
        RCLCPP_INFO(this->get_logger(), "--- Timer Fired: publishMaps() called ---");

        // --- Publish OctoMap (unchanged logic) ---
        if (octree_ && octomap_pub_->get_subscription_count() > 0) {
            octomap_msgs::msg::Octomap msg;
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = target_frame_;

            if (octomap_msgs::binaryMapToMsg(*octree_, msg)) {
                octomap_pub_->publish(msg);
            }
        }
        
        // --- Publish Merged Point Cloud (NEW, more robust logic) ---
        
        // Create an empty message first. This ensures the topic is advertised even if there's no data yet.
        sensor_msgs::msg::PointCloud2 merged_cloud_msg;
        bool has_points = false;

        // Use a lock to safely access the point cloud data
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            
            // Check if the cloud has points
            if (!merged_cloud_pcl_->empty()) {
                pcl::toROSMsg(*merged_cloud_pcl_, merged_cloud_msg);
                has_points = true;
                // Clear the accumulator for the next cycle
                merged_cloud_pcl_->clear(); 
            }
        } // Mutex is released here

        // Only publish if there were points to convert.
        if (has_points) {
            merged_cloud_msg.header.stamp = this->get_clock()->now();
            merged_cloud_msg.header.frame_id = target_frame_;
            merged_cloud_pub_->publish(merged_cloud_msg);
            RCLCPP_INFO(this->get_logger(), "SUCCESS: Published merged point cloud with %ld points.", merged_cloud_msg.width * merged_cloud_msg.height);
        } else {
            RCLCPP_INFO(this->get_logger(), "SKIPPED: No points in merged cloud to publish this cycle.");
        }
    }

    // Member Variables
    std::vector<std::string> input_topics_;
    std::string target_frame_;
    double resolution_;

    std::shared_ptr<octomap::OcTree> octree_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // ADDED: Member variables for point cloud merging
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_pcl_;
    std::mutex cloud_mutex_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapRaycastingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}