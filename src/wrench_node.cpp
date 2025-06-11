// Wrench preprocessing node - applies bias removal and low-pass filtering to F/T sensor data
// Subscribes to raw wrench data and publishes filtered wrench for admittance control

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "admittance_node_types.hpp"  // For Vector6d type

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace ur_admittance_controller {

class WrenchNode : public rclcpp::Node {
public:
  WrenchNode()
  : Node("wrench_node"),
    force_bias_initialized_(false),
    force_bias_(Vector6d::Zero()),
    filtered_wrench_(Vector6d::Zero()),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    first_msg_(true)
  {
    // Declare parameters with defaults
    this->declare_parameter<double>("filter_coefficient", 0.8);
    this->declare_parameter<double>("min_motion_threshold", 1.5);
    this->declare_parameter<bool>("auto_bias_on_start", true);
    
    // Get parameters
    filter_alpha_ = this->get_parameter("filter_coefficient").as_double();
    min_threshold_ = this->get_parameter("min_motion_threshold").as_double();
    auto_bias_ = this->get_parameter("auto_bias_on_start").as_bool();
    
    // Use SensorDataQoS for F/T sensor topics (best practice for sensor streams)
    auto sensor_qos = rclcpp::SensorDataQoS();
    
    // Subscribe to raw F/T sensor data (now from tool_payload frame)
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench_tcp_tool_payload_raw",  // Raw wrench topic in tool_payload frame
      sensor_qos,
      std::bind(&WrenchNode::wrench_callback, this, std::placeholders::_1));
    
    // Publish filtered wrench data
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/wrench_tcp_base",  // Filtered output topic
      sensor_qos);
    
    // Service to reset bias (allows runtime recalibration)
    bias_reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/reset_bias",
      std::bind(&WrenchNode::reset_bias_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), 
                "Wrench node started - Filter alpha: %.2f, Threshold: %.2f N/Nm",
                filter_alpha_, min_threshold_);
  }

private:
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg)
  {
    // Extract 6-DOF wrench data (in tool_payload frame)
    Vector6d wrench_tool_frame{msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                               msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z};
    
    // Transform wrench from tool_payload frame to base_link frame
    Vector6d raw_wrench;
    try {
      // Get transform from tool_payload to base_link
      geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform("base_link", "tool_payload", 
                                   tf2::TimePointZero);
      
      // Extract rotation and translation
      Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped);
      Eigen::Matrix3d rotation = transform.rotation();
      Eigen::Vector3d translation = transform.translation();
      
      // Apply adjoint transformation for wrench (mathematically correct)
      // This accounts for torques created by forces at a distance (lever arm effect)
      Eigen::Vector3d force_tool = wrench_tool_frame.head<3>();
      Eigen::Vector3d torque_tool = wrench_tool_frame.tail<3>();
      
      // Transform force and torque using adjoint
      Eigen::Vector3d force_base = rotation * force_tool;
      Eigen::Vector3d torque_base = rotation * torque_tool + translation.cross(force_base);
      
      // Combine into wrench vector
      raw_wrench.head<3>() = force_base;
      raw_wrench.tail<3>() = torque_base;
      
    } catch (const tf2::TransformException& ex) {
      // If transform not available, warn and use raw data
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Could not transform wrench from tool_payload to base_link: %s", 
                           ex.what());
      raw_wrench = wrench_tool_frame;  // Fallback to raw data
    }
    
    // Initialize bias on first reading if auto_bias is enabled
    if (!force_bias_initialized_ && auto_bias_) {
      force_bias_ = raw_wrench;
      force_bias_initialized_ = true;
      RCLCPP_INFO_ONCE(this->get_logger(), 
                       "Force bias initialized automatically from first reading");
    }
    
    // Apply bias compensation
    Vector6d wrench_compensated = raw_wrench - force_bias_;
    
    // Apply exponential moving average (EMA) low-pass filter
    filtered_wrench_ = first_msg_ ? wrench_compensated : 
                       filter_alpha_ * wrench_compensated + (1.0 - filter_alpha_) * filtered_wrench_;
    first_msg_ = false;
    
    // Apply deadband/threshold - zero out small forces to reduce noise
    Vector6d output_wrench = filtered_wrench_;
    bool above_threshold = false;
    for (size_t i = 0; i < 6; ++i) {
      if (std::abs(output_wrench(i)) > min_threshold_) {
        above_threshold = true;
      } else {
        output_wrench(i) = 0.0;  // Apply deadband per axis
      }
    }
    
    // Only publish if any axis is above threshold
    if (above_threshold || !force_bias_initialized_) {
      // Create output message
      auto filtered_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
      filtered_msg->header = msg->header;  // Preserve timestamp
      filtered_msg->header.frame_id = "base_link";  // Update frame_id to base_link
      
      auto& w = filtered_msg->wrench;
      w.force.x = output_wrench(0); w.force.y = output_wrench(1); w.force.z = output_wrench(2);
      w.torque.x = output_wrench(3); w.torque.y = output_wrench(4); w.torque.z = output_wrench(5);
      
      wrench_pub_->publish(std::move(filtered_msg));
    }
  }
  
  void reset_bias_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // Unused
    
    // Reset bias on next reading
    force_bias_initialized_ = false;
    first_msg_ = true;
    
    response->success = true;
    response->message = "Force bias will be reset on next sensor reading";
    
    RCLCPP_DEBUG(this->get_logger(), "Force bias reset service called");
  }
  
  // Filter state (declared first to match constructor initialization order)
  bool force_bias_initialized_;
  Vector6d force_bias_;
  Vector6d filtered_wrench_;
  
  // TF2 components for frame transformation
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  bool first_msg_;
  
  // Subscriptions and publishers
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bias_reset_service_;
  
  // Parameters
  double filter_alpha_;      // EMA filter coefficient (0.0-1.0)
  double min_threshold_;     // Minimum force/torque threshold
  bool auto_bias_;          // Auto-initialize bias on startup
};

}  // namespace ur_admittance_controller

// Main entry point
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::WrenchNode>();
  
  // Node startup logged in constructor
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}