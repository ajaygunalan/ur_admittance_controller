#pragma once

// UR Admittance Controller - 6-DOF Force-Compliant Motion Control for Universal Robots
// Implements real-time admittance control: M*accel + D*vel + K*pos = F_external

#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include "ur_admittance_controller/ur_admittance_controller_parameters.hpp"
#include <utilities/error.hpp>
#include <utilities/types.hpp>
#include <utilities/conversions.hpp>
#include <utilities/file_io.hpp>
#include <utilities/kinematics.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>

namespace ur_admittance_controller {

// Main admittance control node providing 6-DOF force-compliant robot motion
// Subscribes to F/T sensor data and publishes joint trajectory commands
class AdmittanceNode : public rclcpp::Node {
public:
  explicit AdmittanceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AdmittanceNode() = default;

  void ControlCycle();
  void Initialize();  // Tier 2: Throws on setup failure

private:
  // Callbacks
  void WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
  void JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void DesiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  // Initialization
  Status LoadKinematics();
  void InitializeParameters();
  void InitializeStateVectors();
  void SetupROSInterfaces();
  void SetDefaultEquilibrium();

  // Core algorithms
  void ComputeAdmittance();
  void ComputePoseError();
  void UpdateAdmittanceParameters();
  void ComputeAndPubJointVelocities();
  void LimitToWorkspace();

  // ROS2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  // Dynamic parameter management system
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
  // Robot state variables (Drake notation: q, q_dot)
  std::vector<double> q_current_;
  std::vector<double> q_dot_cmd_;
  bool kinematics_initialized_ = false;

  // Core admittance control state vectors (6-DOF: xyz + rpy)
  Vector6d F_P_B;                    // External forces/torques at Payload in Base frame
  Vector6d V_P_B_commanded;          // Commanded Cartesian velocity at Payload in Base frame

  // Admittance equation matrices: M*accel + D*vel + K*pos = F_external
  Vector6d M_inverse_diag;      // Diagonal elements of mass^-1
  Vector6d D_diag;              // Diagonal elements of damping
  Vector6d K_diag;              // Diagonal elements of stiffness

  // Pose representations
  Eigen::Isometry3d X_BP_current;    // Current transform from Base to Payload
  Eigen::Isometry3d X_BP_desired;    // Target transform from Base to Payload
  Vector6d X_BP_error;               // Payload pose error
  std_msgs::msg::Float64MultiArray velocity_msg_;

  // Workspace and velocity limits
  Vector6d workspace_limits_;  // [x_min, x_max, y_min, y_max, z_min, z_max]
  double arm_max_vel_;
  double arm_max_acc_;
  double admittance_ratio_;    // Scaling factor for external wrench (0-1)

  // KDL kinematics
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;      // Base-to-tip kinematic chain
  KDL::Frame X_W3P;           // Fixed transform from Wrist3 to Payload
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

  // Cached values
  size_t num_joints_ = 0;
  KDL::JntArray q_kdl_;
  KDL::JntArray v_kdl_;
  KDL::Frame X_BW3;

  void MapJointStates(const sensor_msgs::msg::JointState& msg);

public:
  rclcpp::Duration control_period_{std::chrono::milliseconds(10)};  // 100Hz

private:
  bool joint_states_received_ = false;
  std::unordered_map<std::string, size_t> joint_name_to_index_;
  void GetXBPCurrent();
};

}  // namespace ur_admittance_controller
