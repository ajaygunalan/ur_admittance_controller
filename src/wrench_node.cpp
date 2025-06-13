// Wrench preprocessing node - applies bias removal and low-pass filtering to F/T sensor data
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "admittance_node_types.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace ur_admittance_controller {

class WrenchNode : public rclcpp::Node {
public:
  WrenchNode() : Node("wrench_node"),
    force_bias_initialized_(false), tf_ready_(false), first_msg_(true),
    force_bias_(Vector6d::Zero()), filtered_wrench_(Vector6d::Zero()),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true))
  {
    // Hardcoded values - no need for runtime changes
    filter_alpha_ = 0.8;      // EMA filter coefficient
    min_threshold_ = 1.5;     // Minimum force/torque threshold [N/Nm]
    auto_bias_ = true;        // Auto remove bias on startup
    
    auto sensor_qos = rclcpp::SensorDataQoS();
    
    // Setup pub/sub/service
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench_tcp_tool_payload_raw", sensor_qos,
      std::bind(&WrenchNode::wrench_callback, this, std::placeholders::_1));
    
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/wrench_tcp_base", sensor_qos);
    
    bias_reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/reset_bias", 
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;  // Unused parameter
        force_bias_initialized_ = false; 
        first_msg_ = true;
        response->success = true; 
        response->message = "Force bias will be reset on next sensor reading";
      });
    
    RCLCPP_INFO(this->get_logger(), "Started - Filter: 0.8, Threshold: 1.5 N/Nm");
    
    // TF availability check timer
    tf_init_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (tf_buffer_->canTransform("base_link", "tool_payload", tf2::TimePointZero,
                                    std::chrono::seconds(1))) {
        tf_ready_ = true; tf_init_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "TF ready - processing wrench data");
      }
    });
  }

private:
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
    if (!tf_ready_) return;
    
    Vector6d wrench_tool{msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                        msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z};
    Vector6d raw_wrench;
    try {
      auto tf = tf_buffer_->lookupTransform("base_link", "tool_payload", 
                                           tf2::TimePointZero, std::chrono::milliseconds(10));
      Eigen::Isometry3d T = tf2::transformToEigen(tf.transform);
      
      // Adjoint transformation
      raw_wrench.head<3>() = T.rotation() * wrench_tool.head<3>();
      raw_wrench.tail<3>() = T.rotation() * wrench_tool.tail<3>() + 
                             T.translation().cross(raw_wrench.head<3>());
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "TF transform failed: %s", ex.what());
      return;
    }
    
    // Auto-bias on first reading
    if (!force_bias_initialized_ && auto_bias_) {
      force_bias_ = raw_wrench; force_bias_initialized_ = true;
      RCLCPP_INFO_ONCE(this->get_logger(), "Bias initialized");
    }
    
    // Filter and threshold
    Vector6d compensated = raw_wrench - force_bias_;
    filtered_wrench_ = first_msg_ ? compensated : 
                       filter_alpha_ * compensated + (1.0 - filter_alpha_) * filtered_wrench_;
    first_msg_ = false;
    
    Vector6d output = filtered_wrench_;
    bool publish = false;
    for (int i = 0; i < 6; ++i) {
      if (std::abs(output[i]) > min_threshold_) publish = true;
      else output[i] = 0.0;
    }
    
    if (publish || !force_bias_initialized_) {
      auto out_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
      out_msg->header = msg->header;
      out_msg->header.frame_id = "base_link";
      out_msg->wrench.force.x = output[0]; out_msg->wrench.force.y = output[1]; 
      out_msg->wrench.force.z = output[2]; out_msg->wrench.torque.x = output[3];
      out_msg->wrench.torque.y = output[4]; out_msg->wrench.torque.z = output[5];
      wrench_pub_->publish(std::move(out_msg));
    }
  }
  
  // State variables
  bool force_bias_initialized_, tf_ready_, first_msg_;
  Vector6d force_bias_, filtered_wrench_;
  double filter_alpha_, min_threshold_;
  bool auto_bias_;
  
  // ROS interfaces
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr tf_init_timer_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bias_reset_service_;
};

}  // namespace ur_admittance_controller

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ur_admittance_controller::WrenchNode>());
  rclcpp::shutdown();
  return 0;
}