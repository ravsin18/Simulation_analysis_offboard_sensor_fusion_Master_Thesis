#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <vector>
#include <map>

// A struct to hold the specific offset for a single sensor.
struct FixedOffsetParams {
    std::vector<double> pos;
    std::vector<double> rot;
};

class NoiseInjectorNode : public rclcpp::Node
{
public:
    NoiseInjectorNode()
    : Node("noise_injector_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Declare the top-level parameters
        this->declare_parameter("reference_frame", "map");
        auto noisy_frames = this->declare_parameter("noisy_sensor_frames", std::vector<std::string>({}));

        // Now, declare the hierarchical parameters for each frame
        for (const auto& frame : noisy_frames) {
            this->declare_parameter("offsets." + frame + ".pos", std::vector<double>({0.0, 0.0, 0.0}));
            this->declare_parameter("offsets." + frame + ".rot", std::vector<double>({0.0, 0.0, 0.0}));
        }

        // --- Get all parameters ---
        reference_frame_ = this->get_parameter("reference_frame").as_string();
        for (const auto& frame : noisy_frames) {
            offset_configs_[frame] = FixedOffsetParams{
                this->get_parameter("offsets." + frame + ".pos").as_double_array(),
                this->get_parameter("offsets." + frame + ".rot").as_double_array()
            };
            RCLCPP_INFO(this->get_logger(), "Loaded fixed offset for frame '%s'", frame.c_str());
        }
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&NoiseInjectorNode::apply_fixed_offset, this));
            
        RCLCPP_INFO(this->get_logger(), "Noise Injector initialized for %zu frames with unique offsets.",
            offset_configs_.size());
    }

private:
    void apply_fixed_offset()
    {
        // This part of the code remains the same as it was already correct
        for (const auto& [frame_name, params] : offset_configs_) {
            try {
                geometry_msgs::msg::TransformStamped clean_transform = 
                    tf_buffer_.lookupTransform(reference_frame_, frame_name, tf2::TimePointZero);

                geometry_msgs::msg::TransformStamped noisy_transform = clean_transform;
                
                noisy_transform.transform.translation.x += params.pos[0];
                noisy_transform.transform.translation.y += params.pos[1];
                noisy_transform.transform.translation.z += params.pos[2];
                
                tf2::Quaternion q_orig, q_offset;
                tf2::fromMsg(noisy_transform.transform.rotation, q_orig);
                q_offset.setRPY(params.rot[0], params.rot[1], params.rot[2]);
                q_orig = q_offset * q_orig;
                q_orig.normalize();
                noisy_transform.transform.rotation = tf2::toMsg(q_orig);

                noisy_transform.child_frame_id = frame_name + "_noisy";
                noisy_transform.header.stamp = this->get_clock()->now();
                tf_broadcaster_->sendTransform(noisy_transform);

            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_ONCE(this->get_logger(), "Could not get transform for '%s': %s", ex.what());
            }
        }
    }

    // Member Variables
    std::string reference_frame_;
    std::map<std::string, FixedOffsetParams> offset_configs_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoiseInjectorNode>());
    rclcpp::shutdown();
    return 0;
}