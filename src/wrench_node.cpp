// Wrench preprocessing node - applies bias removal and low-pass filtering to F/T sensor data
// Subscribes to raw wrench data and publishes filtered wrench for admittance control

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "admittance_node_types.hpp"  // For Vector6d type

namespace ur_admittance_controller {

class WrenchNode : public rclcpp::Node {
public:
  WrenchNode()
  : Node("wrench_node"),
    force_bias_initialized_(false),
    force_bias_(Vector6d::Zero()),
    filtered_wrench_(Vector6d::Zero()),
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
    
    // Subscribe to raw F/T sensor data
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench_tcp_base_raw",  // Raw wrench topic
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
    // Extract 6-DOF wrench data (already in base frame for UR5e)
    Vector6d raw_wrench;
    raw_wrench << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                  msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
    
    // Initialize bias on first reading if auto_bias is enabled
    if (!force_bias_initialized_ && auto_bias_) {
      force_bias_ = raw_wrench;
      force_bias_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), 
                  "Force bias initialized: F=[%.2f, %.2f, %.2f] N, T=[%.2f, %.2f, %.2f] Nm",
                  force_bias_(0), force_bias_(1), force_bias_(2),
                  force_bias_(3), force_bias_(4), force_bias_(5));
    }
    
    // Apply bias compensation
    Vector6d wrench_compensated = raw_wrench - force_bias_;
    
    // Apply exponential moving average (EMA) low-pass filter
    // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    if (first_msg_) {
      filtered_wrench_ = wrench_compensated;
      first_msg_ = false;
    } else {
      filtered_wrench_ = filter_alpha_ * wrench_compensated + 
                         (1.0 - filter_alpha_) * filtered_wrench_;
    }
    
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
      filtered_msg->header = msg->header;  // Preserve timestamp and frame_id
      
      filtered_msg->wrench.force.x = output_wrench(0);
      filtered_msg->wrench.force.y = output_wrench(1);
      filtered_msg->wrench.force.z = output_wrench(2);
      filtered_msg->wrench.torque.x = output_wrench(3);
      filtered_msg->wrench.torque.y = output_wrench(4);
      filtered_msg->wrench.torque.z = output_wrench(5);
      
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
    
    RCLCPP_INFO(this->get_logger(), "Force bias reset requested");
  }
  
  // Subscriptions and publishers
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bias_reset_service_;
  
  // Filter state
  bool force_bias_initialized_;
  Vector6d force_bias_;
  Vector6d filtered_wrench_;
  bool first_msg_;
  
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
  
  RCLCPP_INFO(node->get_logger(), 
              "Starting wrench node - filtering F/T sensor data");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}