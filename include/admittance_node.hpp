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
#include <std_msgs/msg/float64_multi_array.hpp>
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
  bool initialize();

  // Publishing functions (ROS1 style organization)
  void publish_arm_state_in_world();
  void publish_debugging_signals();

 private:
  // ROS2 callback functions for sensor data processing
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void desired_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void robot_description_callback(const std_msgs::msg::String::ConstSharedPtr msg);
  
  // System initialization and setup
  bool load_kinematics();
  bool initialize_desired_pose();
  void wait_for_transformations();
  
  // Initialization sequence methods
  void wait_for_robot_ready();
  void wait_for_kinematics();
  void wait_for_initial_pose();
  
  // Core admittance control algorithms
  bool compute_admittance();
  Vector6d compute_pose_error();
  void update_mass_matrix();
  void update_damping_matrix();
  void update_stiffness_matrix();
  void update_admittance_parameters();
  
  // Coordinate transformations and motion processing
  bool compute_joint_velocities(const Vector6d& cartesian_velocity);
  void send_commands_to_robot();
  
  // Safety validation and limits
  bool check_pose_limits(const Vector6d& pose_error);
  void limit_to_workspace();
  void limit_joint_velocities();
  void apply_admittance_ratio(double ratio);
  
  // Transform utilities for coordinate frame conversions
  void getEndEffectorPose(Eigen::Isometry3d& pose);
  bool get_transform_matrix(Eigen::Isometry3d& transform,
                           const std::string& from_frame,
                           const std::string& to_frame,
                           const std::chrono::milliseconds& timeout = std::chrono::milliseconds(50));
  bool get_rotation_matrix_6d(Matrix6d& rotation_matrix,
                             const std::string& from_frame,
                             const std::string& to_frame);
  // ROS2 communication interfaces
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  // Dynamic parameter management system
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
  // TF2 transform system for coordinate frame management
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  // Robot state variables with thread-safe access (Drake notation: q, q_dot)
  std::vector<double> q_current_;            // Current sensor joint positions
  std::vector<double> q_dot_current_;        // Current sensor joint velocities
  // q_cmd_ removed - velocity controller doesn't need position integration  
  std::vector<double> q_dot_cmd_;            // Computed command joint velocities
  geometry_msgs::msg::WrenchStamped current_wrench_;
  // URDF robot model storage
  std::string robot_description_;
  bool kinematics_initialized_ = false;
  // Reference pose management for admittance control
  bool desired_pose_initialized_ = false;
  bool joint_states_received_ = false;  // Track if robot is loaded
  // Core admittance control state vectors (6-DOF: xyz + rpy) 
  Vector6d Wrench_tcp_base_;         // External forces/torques in base frame (filtered & bias-compensated)
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
  
  // Control loop period (elegant like ROS1's expectedCycleTime)
  rclcpp::Duration control_period_{std::chrono::milliseconds(10)};  // 100Hz default
  
  // Compute forward kinematics from joint positions (industry standard naming)
  void computeForwardKinematics();
};

}  // namespace ur_admittance_controller