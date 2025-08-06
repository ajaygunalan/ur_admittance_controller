#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include "ur_admittance_controller/ur_admittance_controller_parameters.hpp"
#include <utilities/error.hpp>
#include <utilities/types.hpp>
#include <utilities/conversions.hpp>
#include <utilities/kinematics.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>

namespace ur_admittance_controller {

class AdmittanceNode : public rclcpp::Node {
public:
  explicit AdmittanceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  void ControlCycle();
  void configure();

private:
  void WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
  void JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void DesiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  Status LoadKinematics();
  void ComputeAdmittance();
  void ComputePoseError();
  void ComputeAndPubJointVelocities();
  void LimitToWorkspace();
  void GetXBPCurrent();

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
  std::vector<double> q_current_;
  std::vector<double> q_dot_cmd_;

  Vector6d F_P_B = Vector6d::Zero();
  Vector6d V_P_B_commanded = Vector6d::Zero();

  Vector6d M_inverse_diag;
  Vector6d D_diag;
  Vector6d K_diag;

  Eigen::Isometry3d X_BP_current = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d X_BP_desired = Eigen::Isometry3d::Identity();
  Vector6d X_BP_error = Vector6d::Zero();
  std_msgs::msg::Float64MultiArray velocity_msg_;

  Vector6d workspace_limits_;
  double arm_max_vel_;
  double arm_max_acc_;
  double admittance_ratio_;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::Frame X_W3P;
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

  size_t num_joints_ = 0;
  KDL::JntArray q_kdl_;
  KDL::JntArray v_kdl_;
  KDL::Frame X_BW3;

public:
  rclcpp::Duration control_period_{std::chrono::milliseconds(10)};
  bool joint_states_received_ = false;

private:
  std::unordered_map<std::string, size_t> joint_name_to_index_;
};

}  // namespace ur_admittance_controller