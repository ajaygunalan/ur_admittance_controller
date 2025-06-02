#pragma once

// UR Admittance Controller - 6-DOF Force-Compliant Motion Control for Universal Robots
// Implements real-time admittance control: M*accel + D*vel + K*pos = F_external

// Related header
#include "ur_admittance_controller/ur_admittance_controller_parameters.hpp"
#include "admittance_node_types.hpp"
#include "admittance_constants.hpp"

// Standard library headers
#include <atomic>
#include <chrono>
#include <memory>
#include <vector>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

// Third-party kinematics libraries
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <urdf/model.h>

namespace ur_admittance_controller {

// Main admittance control node providing 6-DOF force-compliant robot motion
// Subscribes to F/T sensor data and publishes joint trajectory commands
class AdmittanceNode : public rclcpp::Node {
 public:
  explicit AdmittanceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AdmittanceNode() = default;

 private:
  // ROS2 callback functions for sensor data processing
  void WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
  void JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void DesiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  // System initialization and setup
  bool LoadKinematics();
  bool InitializeDesiredPose();
  // Core admittance control algorithms
  bool ComputeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out);
  Vector6d ComputePoseError_tip_base();
  void UpdateMassMatrix();
  void UpdateDampingMatrix();
  void UpdateStiffnessMatrix();
  void UpdateAdmittanceMatrices();
  // Coordinate transformations and motion processing
  bool ConvertToJointSpace(const Vector6d& cartesian_velocity, const rclcpp::Duration& period);
  bool CheckDeadband();
  // Main control loop execution
  bool UnifiedControlStep(double dt);
  bool ValidatePoseErrorSafety(const Vector6d& pose_error);
  // Transform utilities for coordinate frame conversions
  Vector6d TransformWrench(const Vector6d& wrench_sensor_frame);
  bool GetCurrentEndEffectorPose(Eigen::Isometry3d& pose);
  // ROS2 communication interfaces
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  // Control timer (100Hz) for trajectory streaming admittance control
  rclcpp::TimerBase::SharedPtr control_timer_;
  void ControlTimerCallback();
  // Dynamic parameter management system
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;
  // TF2 transform system for coordinate frame management
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  // Robot state variables with thread-safe access (Drake notation: q, q_dot)
  std::vector<double> q_current_;            // Current sensor joint positions
  std::vector<double> q_dot_current_;        // Current sensor joint velocities
  std::vector<double> q_cmd_;                // Integrated command positions  
  std::vector<double> q_dot_cmd_;            // Computed command joint velocities
  geometry_msgs::msg::WrenchStamped current_wrench_;
  // Note: URDF robot model obtained directly from parameter server (no caching needed)
  // Reference pose management for admittance control
  bool desired_pose_initialized_ = false;
  // Core admittance control state vectors (6-DOF: xyz + rpy) 
  Vector6d Wrench_tcp_base_;         // External forces/torques in base frame (filtered)
  Vector6d V_tcp_base_commanded_;    // Commanded Cartesian velocity output
  Vector6d V_tcp_base_desired_;      // Internal velocity state from admittance equation
  // Admittance equation matrices: M*accel + D*vel + K*pos = F_external
  Matrix6d M_;                  // Virtual mass matrix (6x6)
  Matrix6d M_inverse_;          // Precomputed mass inverse
  Matrix6d D_;                  // Damping matrix
  Matrix6d K_;                  // Stiffness matrix
  // Optimized diagonal storage for performance-critical computations
  Vector6d M_inverse_diag_;     // Diagonal elements of mass^-1
  Vector6d D_diag_;             // Diagonal elements of damping
  Vector6d K_diag_;             // Diagonal elements of stiffness
  // Pose representations for admittance control
  Eigen::Isometry3d X_tcp_base_current_;   // Current TCP pose
  Eigen::Isometry3d X_tcp_base_desired_;   // Target reference TCP pose
  // Intermediate computation variables
  Vector6d error_tcp_base_;     // Pose error (desired - current)
  // Pre-allocated ROS2 messages to avoid real-time allocations
  trajectory_msgs::msg::JointTrajectory trajectory_msg_;
  // KDL kinematics for inverse velocity solving
  KDL::Tree kdl_tree_;                                        // Full robot kinematic tree
  KDL::Chain kdl_chain_;                                      // Base-to-tip kinematic chain
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_; // WDLS velocity solver
  bool kinematics_ready_ = false;
  // Control loop timing - member variable for thread safety
  std::chrono::steady_clock::time_point last_control_time_;
};

}  // namespace ur_admittance_controller