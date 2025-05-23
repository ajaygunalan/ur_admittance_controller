#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <Eigen/Dense>

namespace ur_admittance {

/**
 * @brief Node to generate simulated wrench (force/torque) signals for testing admittance controllers.
 *
 * Accurately simulates a UR5e force/torque sensor by:
 * - Generating forces in the tool frame
 * - Transforming them to the base frame (like real UR5e sensors)
 * - Simulating and compensating for tool weight (gravity compensation)
 * - Providing a zeroing service similar to zero_ftsensor() in real UR robots
 */
class WrenchSignalGenerator : public rclcpp::Node {
public:
  explicit WrenchSignalGenerator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void timer_callback();

  // --- ROS Interfaces ---
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_service_;
  
  // --- TF ---
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Parameters ---
  double frequency_{};
  double amplitude_{};
  double publish_rate_{};
  std::string tool_frame_id_;  // Frame where forces are generated (tool frame)
  std::string base_frame_id_;  // Frame for output (base frame)
  double tool_mass_{};         // Mass of the tool (kg)
  bool apply_gravity_comp_{};  // Whether to apply gravity compensation

  // --- State ---
  double time_{};
  Eigen::Vector3d tool_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_{0.0, 0.0, -9.81};
  Eigen::Vector3d wrench_offset_{Eigen::Vector3d::Zero()};
  
  // --- Methods ---
  void zero_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  Eigen::Matrix<double, 6, 1> generateWrench();
  Eigen::Matrix<double, 6, 1> applyGravityCompensation(const Eigen::Matrix<double, 6, 1>& wrench);
  Eigen::Matrix<double, 6, 1> transformWrench(const Eigen::Matrix<double, 6, 1>& wrench_tool);
};

} // namespace ur_admittance