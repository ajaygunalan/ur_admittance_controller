#include "init_robot.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <thread>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <urdf/model.h>

namespace ur_admittance_controller {

namespace fs = std::filesystem;
using namespace constants;

// ============================================================================
// Pure Functions (No Side Effects) - Implementations
// ============================================================================

KinematicModel build_kinematic_model(const std::string& urdf_string) {
  urdf::Model urdf_model;
  urdf_model.initString(urdf_string);
  
  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel(urdf_model, tree);
  
  KDL::Chain robot_chain;
  tree.getChain("base_link", "wrist_3_link", robot_chain);
  
  KDL::Chain tool_chain;
  tree.getChain("wrist_3_link", "p42v_link1", tool_chain);
  
  KDL::Frame tool_offset = KDL::Frame::Identity();
  for (unsigned int i = 0; i < tool_chain.getNrOfSegments(); ++i)
    tool_offset = tool_offset * tool_chain.getSegment(i).getFrameToTip();
  
  return KinematicModel{
    robot_chain,
    tool_offset, 
    robot_chain.getNrOfJoints()
  };
}

CartesianPose compute_forward_kinematics(const KinematicModel& model,
                                         const JointConfiguration& joints) {
  KDL::ChainFkSolverPos_recursive fk_solver(model.robot_chain);
  
  KDL::JntArray kdl_joints(6);
  for (size_t i = 0; i < 6; ++i)
    kdl_joints(i) = joints[i];
  
  KDL::Frame wrist_frame;
  fk_solver.JntToCart(kdl_joints, wrist_frame);
  
  KDL::Frame end_effector_frame = wrist_frame * model.tool_offset;
  
  double qx, qy, qz, qw;
  end_effector_frame.M.GetQuaternion(qx, qy, qz, qw);
  
  return CartesianPose{
    {end_effector_frame.p.x(), end_effector_frame.p.y(), end_effector_frame.p.z()},
    {qw, qx, qy, qz}
  };
}

std::vector<TrajectoryPoint> create_trajectory(const JointConfiguration& start,
                                               const JointConfiguration& target,
                                               rclcpp::Duration duration) {
  return {
    {start, rclcpp::Duration::from_seconds(0.0)},
    {target, duration}
  };
}

YAML::Node build_equilibrium_config(const CartesianPose& pose) {
  YAML::Node config;
  
  config["admittance_node"]["ros__parameters"]["equilibrium.position"] = 
    std::vector<double>(pose.position.begin(), pose.position.end());
  
  config["admittance_node"]["ros__parameters"]["equilibrium.orientation"] = 
    std::vector<double>(pose.orientation.begin(), pose.orientation.end());
  
  return config;
}

std::optional<JointConfiguration> parse_joint_state(const sensor_msgs::msg::JointState& msg) {
  JointConfiguration result;
  
  for (size_t i = 0; i < 6; ++i) {
    auto it = std::find(msg.name.begin(), msg.name.end(), JOINT_NAMES[i]);
    if (it == msg.name.end())
      return std::nullopt;
    
    size_t index = std::distance(msg.name.begin(), it);
    if (index >= msg.position.size())
      return std::nullopt;
    
    result.values[i] = msg.position[index];
  }
  
  return result;
}

fs::path get_config_path() {
  const char* workspace = std::getenv("ROS_WORKSPACE");
  fs::path base = workspace ? fs::path(workspace) : 
                             fs::path(std::getenv("HOME")) / "ros2_ws";
  
  return base / "src" / "ur_admittance_controller" / "config" / "equilibrium.yaml";
}

// ============================================================================
// IO Operations (Side Effects) - Implementations
// ============================================================================

std::string fetch_robot_description(rclcpp::Node::SharedPtr node) {
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
    node, "/robot_state_publisher");
  
  param_client->wait_for_service(SERVICE_TIMEOUT);
  
  auto params = param_client->get_parameters({"robot_description"});
  return params[0].as_string();
}

JointConfiguration read_current_joints(rclcpp::Node::SharedPtr node) {
  std::optional<JointConfiguration> result;
  
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1,
    [&result](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
      if (!result)
        result = parse_joint_state(*msg);
    });
  
  auto start_time = node->now();
  while (!result && (node->now() - start_time).seconds() < SERVICE_TIMEOUT.count()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(POLLING_INTERVAL);
  }
  
  return result.value();
}

void execute_trajectory(rclcpp::Node::SharedPtr node,
                        const std::vector<TrajectoryPoint>& trajectory) {
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  
  auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
    node, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
  
  action_client->wait_for_action_server(SERVICE_TIMEOUT);
  
  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = std::vector<std::string>(
    JOINT_NAMES.begin(), JOINT_NAMES.end());
  
  goal.trajectory.points.resize(trajectory.size());
  for (size_t i = 0; i < trajectory.size(); ++i) {
    goal.trajectory.points[i].positions.assign(
      trajectory[i].joints.begin(), trajectory[i].joints.end());
    goal.trajectory.points[i].time_from_start = trajectory[i].time_from_start;
  }
  
  auto goal_future = action_client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node, goal_future);
  
  auto goal_handle = goal_future.get();
  auto result_future = action_client->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(node, result_future);
}

void save_yaml_file(const fs::path& path, const YAML::Node& config) {
  fs::create_directories(path.parent_path());
  
  std::ofstream file(path);
  file << config;
}

} // namespace ur_admittance_controller

// ============================================================================
// Main - Pipeline Orchestration
// ============================================================================

int main(int argc, char** argv) {
  using namespace ur_admittance_controller;
  using namespace ur_admittance_controller::constants;
  
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("init_robot");
  
  RCLCPP_INFO(node->get_logger(), "Starting robot initialization...");
  
  // Pipeline: Fetch → Parse → Read → Move → Compute → Save
  
  std::string urdf_string = fetch_robot_description(node);
  KinematicModel model = build_kinematic_model(urdf_string);
  
  JointConfiguration current_joints = read_current_joints(node);
  
  std::vector<TrajectoryPoint> trajectory = create_trajectory(
    current_joints,
    EQUILIBRIUM_JOINTS,
    rclcpp::Duration(TRAJECTORY_DURATION)
  );
  
  RCLCPP_INFO(node->get_logger(), "Moving robot to equilibrium position...");
  execute_trajectory(node, trajectory);
  
  CartesianPose equilibrium_pose = compute_forward_kinematics(model, EQUILIBRIUM_JOINTS);
  
  YAML::Node config = build_equilibrium_config(equilibrium_pose);
  fs::path config_path = get_config_path();
  save_yaml_file(config_path, config);
  
  RCLCPP_INFO(node->get_logger(), 
    "Equilibrium configuration saved to: %s", config_path.string().c_str());
  
  RCLCPP_INFO(node->get_logger(), "Robot initialization completed successfully");
  
  rclcpp::shutdown();
  return 0;
}