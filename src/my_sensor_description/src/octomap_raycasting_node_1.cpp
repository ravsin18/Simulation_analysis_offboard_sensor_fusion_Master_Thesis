#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// ADDED: Headers for PCL cloud and mutex
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>

#include <exception>
#include <string>
#include <vector>
#include <memory>


class OctomapRaycastingNode : public rclcpp::Node
{
public:
    OctomapRaycastingNode()
    : Node("octomap_raycasting_node_1"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Octomap Raycasting Node...");
        
        // Use the robust "declare then get" pattern for all parameters
        this->declare_parameter("target_frame", "map");
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("input_topics", std::vector<std::string>({}));
        
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("resolution", resolution_);
        this->get_parameter("input_topics", input_topics_);

        if (input_topics_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'input_topics' is empty. No data will be processed.");
            return;
        }

        // Initialization
        octree_ = std::make_shared<octomap::OcTree>(resolution_);
        // ADDED: Initialize the PCL point cloud
        merged_cloud_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        for (const auto &topic : input_topics_) {
            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, rclcpp::SystemDefaultsQoS(),
                [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    this->processPointCloud(msg, topic);
                }
            );
            subscriptions_.push_back(sub);
        }

        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/octomap_custom", 10);
        // ADDED: Create the publisher for the merged point cloud
        merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_point_cloud", 10);
        
        // ADDED: A timer to periodically publish all map data
        publish_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&OctomapRaycastingNode::publishMaps, this));

        RCLCPP_INFO(this->get_logger(), "Node initialization complete. Subscribed to %zu topics.", subscriptions_.size());
    }

private:
    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &topic)
    {
        geometry_msgs::msg::TransformStamped transform;
        std::string sensor_frame_id = msg->header.frame_id;
        std::string noisy_frame_id = sensor_frame_id + "_noisy";

        try {
            transform = tf_buffer_.lookupTransform(target_frame_, noisy_frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            try {
                transform = tf_buffer_.lookupTransform(target_frame_, sensor_frame_id, tf2::TimePointZero);
            } catch (const tf2::TransformException &ex2) {
                RCLCPP_WARN(this->get_logger(), "[%s] Could not get transform for frame '%s': %s",
                    topic.c_str(), sensor_frame_id.c_str(), ex2.what());
                return;
            }
        }

        sensor_msgs::msg::PointCloud2 cloud_transformed;
        tf2::doTransform(*msg, cloud_transformed, transform);

        // --- OctoMap Update Logic ---
        octomap::point3d origin(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_transformed, "x"), iter_y(cloud_transformed, "y"), iter_z(cloud_transformed, "z");
             iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            octomap::point3d endpoint(*iter_x, *iter_y, *iter_z);
            if ((origin - endpoint).norm() > 0.01) {
                octree_->insertRay(origin, endpoint);
            }
        }
        
        // --- ADDED: Merged Point Cloud Logic ---
        pcl::PointCloud<pcl::PointXYZ> incoming_cloud_pcl;
        pcl::fromROSMsg(cloud_transformed, incoming_cloud_pcl);
        // Lock the mutex to safely add points to the shared cloud
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        *merged_cloud_pcl_ += incoming_cloud_pcl;
    }

    // MODIFIED: This function now publishes both maps and is called by a timer
    void publishMaps()
    {
        // Publish OctoMap
        if (octree_ && octomap_pub_->get_subscription_count() > 0) {
            octomap_msgs::msg::Octomap msg;
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = target_frame_;
            if (octomap_msgs::binaryMapToMsg(*octree_, msg)) {
                octomap_pub_->publish(msg);
            }
        }

        // ADDED: Publish Merged Point Cloud
        sensor_msgs::msg::PointCloud2 merged_cloud_msg;
        // Use a lock to safely access the point cloud data
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!merged_cloud_pcl_->empty()) {
                pcl::toROSMsg(*merged_cloud_pcl_, merged_cloud_msg);
                // We don't clear the merged cloud here so it's a persistent view
            }
        }

        // Only publish if there were points to convert.
        if (merged_cloud_msg.width * merged_cloud_msg.height > 0) {
            merged_cloud_msg.header.stamp = this->get_clock()->now();
            merged_cloud_msg.header.frame_id = target_frame_;
            merged_cloud_pub_->publish(merged_cloud_msg);
        }
    }

    // Member Variables
    std::string target_frame_;
    double resolution_;
    std::vector<std::string> input_topics_;
    std::shared_ptr<octomap::OcTree> octree_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // ADDED: Member variables for merged cloud and timer
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_pcl_;
    std::mutex cloud_mutex_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapRaycastingNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}