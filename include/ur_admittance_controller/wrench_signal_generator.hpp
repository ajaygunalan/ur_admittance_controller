/**
 * @file wrench_signal_generator.hpp
 * @brief Simulates UR5e force/torque sensor signals for testing admittance controllers
 */

#pragma once

// Standard libraries
#include <string>

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Math libraries
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
  /**
   * @brief Construct a new Wrench Signal Generator
   * 
   * Initializes parameters, TF listener, publisher, service, and timer
   * 
   * @param options Node options for ROS2 node configuration
   */
  explicit WrenchSignalGenerator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /** @name Core Methods
   * @{ */
  /**
   * @brief Timer callback that generates and publishes wrench signals
   * 
   * This function is called at the publish rate and performs these steps:
   * 1. Generate a simulated wrench in tool frame
   * 2. Apply gravity compensation if enabled
   * 3. Transform wrench to base frame
   * 4. Publish wrench message
   */
  void timer_callback();
  
  /**
   * @brief Service callback to zero the force/torque sensor
   * 
   * Simulates the zero_ftsensor() function in real UR robots
   * by storing the current wrench as an offset that will be
   * subtracted from future readings
   * 
   * @param request Empty request
   * @param response Response with success status and message
   */
  void zero_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /** @} */
  
  /** @name Wrench Processing Methods
   * @{ */
  /**
   * @brief Generate a simulated wrench in the tool frame
   * 
   * Creates a sinusoidal force pattern in the X direction
   * with configurable frequency and amplitude
   * 
   * @return 6D wrench vector (3D force, 3D torque)
   */
  Eigen::Matrix<double, 6, 1> generateWrench();
  
  /**
   * @brief Apply gravity compensation to the wrench
   * 
   * Simulates how a real UR5e sensor compensates for the tool weight
   * based on tool mass and center of mass parameters
   * 
   * @param wrench Original wrench to compensate
   * @return Gravity-compensated wrench
   */
  Eigen::Matrix<double, 6, 1> applyGravityCompensation(const Eigen::Matrix<double, 6, 1>& wrench);
  
  /**
   * @brief Transform wrench from tool frame to base frame
   * 
   * Uses TF to transform the wrench according to the current robot pose
   * 
   * @param wrench_tool Wrench in tool frame
   * @return Wrench in base frame
   */
  Eigen::Matrix<double, 6, 1> transformWrench(const Eigen::Matrix<double, 6, 1>& wrench_tool);
  /** @} */

  /** @name ROS Communication
   * @{ */
  // Publisher and timer
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_service_;
  
  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  /** @} */

  /** @name Configuration
   * @{ */
  double frequency_{};          ///< Frequency of the simulated force oscillation (Hz)
  double amplitude_{};          ///< Amplitude of the simulated force (N)
  double publish_rate_{};       ///< Rate at which to publish wrench messages (Hz)
  std::string tool_frame_id_;   ///< Frame where forces are generated (tool frame)
  std::string base_frame_id_;   ///< Frame for output (base frame)
  double tool_mass_{};          ///< Mass of the tool (kg)
  bool apply_gravity_comp_{};   ///< Whether to apply gravity compensation
  /** @} */

  /** @name State
   * @{ */
  double time_{};                                  ///< Time for signal generation (seconds)
  Eigen::Vector3d tool_offset_{Eigen::Vector3d::Zero()}; ///< Tool center of mass offset (m)
  Eigen::Vector3d gravity_{0.0, 0.0, -9.81};      ///< Gravity vector (m/s²)
  Eigen::Vector3d wrench_offset_{Eigen::Vector3d::Zero()}; ///< Zero offset for the wrench
  /** @} */
};

} // namespace ur_admittance