#pragma once

// UR Admittance Controller - 6-DOF Force-Compliant Motion Control for Universal Robots
// Implements real-time admittance control: M*accel + D*vel + K*pos = F_external

// Related header
#include "ur_admittance_controller/ur_admittance_controller_parameters.hpp"
#include "admittance_node_types.hpp"

// Standard library headers
#include <atomic>
#include <chrono>
#include <memory>
#include <vector>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// Third-party kinematics libraries
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>

namespace ur_admittance_controller {

// Main admittance control node providing 6-DOF force-compliant robot motion
// Subscribes to F/T sensor data and publishes joint trajectory commands
class AdmittanceNode : public rclcpp::Node {
 public:
  explicit AdmittanceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AdmittanceNode() = default;
  
  // Main control cycle - called from main loop at 100Hz
  void control_cycle();
  
  // Initialization - must be called before control loop
  void initialize();

 private:
  // ROS2 callback functions for sensor data processing
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void desired_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  
  // System initialization and setup
  bool load_kinematics();
  void initializeParameters();
  void initializeStateVectors();
  void setupROSInterfaces();
  void setDefaultEquilibrium();
  
  // Initialization sequence methods
  bool checkJointStates();
  
  // Core admittance control algorithms
  void compute_admittance();
  Vector6d compute_pose_error();
  void update_admittance_parameters();
  
  // Coordinate transformations and motion processing
  bool compute_joint_velocities(const Vector6d& cartesian_velocity);
  void send_commands_to_robot();
  
  // Safety validation and limits
  void limit_to_workspace();
  
  // ROS2 communication interfaces
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  // Dynamic parameter management system
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
  // Robot state variables with thread-safe access (Drake notation: q, q_dot)
  std::vector<double> q_current_;            // Current sensor joint positions
  // q_dot_current_ removed - joint velocities not used in velocity-based controller
  // q_cmd_ removed - velocity controller doesn't need position integration  
  std::vector<double> q_dot_cmd_;            // Computed command joint velocities
  // Kinematics initialization flag
  bool kinematics_initialized_ = false;
  // Core admittance control state vectors (6-DOF: xyz + rpy) 
  Vector6d Wrench_tcp_base_;         // External forces/torques in base frame (filtered & bias-compensated)
  Vector6d V_tcp_base_commanded_;    // Commanded Cartesian velocity (integrated and limited)
  // Admittance equation matrices: M*accel + D*vel + K*pos = F_external
  // Only diagonal elements are used - removed redundant full matrix storage
  Vector6d M_inverse_diag_;     // Diagonal elements of mass^-1
  Vector6d D_diag_;             // Diagonal elements of damping
  Vector6d K_diag_;             // Diagonal elements of stiffness
  // Pose representations for admittance control
  Eigen::Isometry3d X_tcp_base_current_;   // Current TCP pose
  Eigen::Isometry3d X_tcp_base_desired_;   // Target reference TCP pose
  // Pre-allocated ROS2 messages to avoid real-time allocations
  std_msgs::msg::Float64MultiArray velocity_msg_;
  
  // Workspace and velocity limits (from ROS1)
  Vector6d workspace_limits_;  // [x_min, x_max, y_min, y_max, z_min, z_max]
  double arm_max_vel_;         // Maximum Cartesian velocity
  double arm_max_acc_;         // Maximum Cartesian acceleration
  double admittance_ratio_;    // Scaling factor for external wrench (0-1)
  // KDL kinematics for inverse velocity solving
  KDL::Tree kdl_tree_;                                        // Full robot kinematic tree
  KDL::Chain kdl_chain_;                                      // Base-to-tip kinematic chain
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_; // WDLS velocity solver
  
  // Forward kinematics solver (ROS1-style direct computation)
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  
  // Flag to ensure we compute FK after joint states are updated
  bool joint_states_updated_ = false;
  
  // Cached values to avoid repeated computations in control loop
  size_t num_joints_ = 0;  // Number of joints in kinematic chain
  KDL::JntArray q_kdl_;    // Pre-allocated KDL joint positions
  KDL::JntArray v_kdl_;    // Pre-allocated KDL joint velocities
  
  // Control timer
  rclcpp::TimerBase::SharedPtr control_timer_;
  
 public:
  // Control loop period (elegant like ROS1's expectedCycleTime)
  rclcpp::Duration control_period_{std::chrono::milliseconds(10)};  // 100Hz default
  
 private:
  
  // Compute forward kinematics from joint positions (industry standard naming)
  void computeForwardKinematics();
};

}  // namespace ur_admittance_controller