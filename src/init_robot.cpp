#include <fstream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <utilities/kinematics.hpp>
#include <utilities/logging.hpp>
#include <utilities/constants.hpp>
#include <utilities/types.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace ur_admittance_controller {

// Kinematics helpers
namespace kinematics {
    Result<KinematicsComponents> InitializeFromUrdfString(
        const std::string& urdf_string,
        const std::string& base_link,
        const std::string& tip_link) {
        
        urdf::Model urdf_model;
        if (!urdf_model.initString(urdf_string)) {
            return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                           "Failed to parse URDF string"));
        }
        
        KinematicsComponents components;
        
        // Convert URDF to KDL tree
        if (!kdl_parser::treeFromUrdfModel(urdf_model, components.tree)) {
            return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                           "Failed to convert URDF to KDL tree"));
        }
        
        // Extract chain from base to wrist (tool0 = wrist_3_link for UR robots)
        if (!components.tree.getChain(base_link, "wrist_3_link", components.robot_chain)) {
            return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                fmt::format("Failed to extract chain from {} to wrist_3_link", base_link)));
        }
        
        // Get transform from wrist to actual tip (through all fixed joints and sensor frames)
        KDL::Chain tool_chain;
        if (!components.tree.getChain("wrist_3_link", tip_link, tool_chain)) {
            // If direct chain fails, just use identity (wrist_3 = tool0)
            components.tool_offset = KDL::Frame::Identity();
        } else {
            // Compute full transform through all segments (including sensor frames)
            components.tool_offset = KDL::Frame::Identity();
            for (unsigned int i = 0; i < tool_chain.getNrOfSegments(); ++i) {
                components.tool_offset = components.tool_offset * tool_chain.getSegment(i).getFrameToTip();
            }
        }
        
        components.num_joints = components.robot_chain.getNrOfJoints();
        
        RCLCPP_DEBUG(rclcpp::get_logger("kinematics"),
                     "Initialized KDL: %zu joints, %d segments",
                     components.num_joints, components.robot_chain.getNrOfSegments());
        
        return components;
    }

    Transform KdlToEigen(const KDL::Frame& frame) {
        Transform result;
        result.translation() << frame.p.x(), frame.p.y(), frame.p.z();
        
        double x, y, z, w;
        frame.M.GetQuaternion(x, y, z, w);
        result.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
        
        return result;
    }
}

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using namespace constants;
using namespace CalibrationConstants;

auto LoadKinematics(rclcpp::Node::SharedPtr node) {
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/robot_state_publisher");
  if (!client->wait_for_service(SERVICE_TIMEOUT)) {
    RCLCPP_FATAL(node->get_logger(), "robot_state_publisher not available");
    std::exit(1);
  }
  auto kin = kinematics::InitializeFromUrdfString(
    client->get_parameters({"robot_description"})[0].as_string(), "base_link", "p42v_link1");
  if (!kin) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize kinematics: %s", kin.error().message.c_str());
    std::exit(1);
  }
  RCLCPP_INFO(node->get_logger(), "Kinematics initialized with %zu joints", kin.value().num_joints);
  return kin.value();
}

void MoveToEquilibrium(rclcpp::Node::SharedPtr node,
                       rclcpp::executors::SingleThreadedExecutor& executor,
                       const JointVector& current,
                       const JointVector& target,
                       double duration) {
  auto action = rclcpp_action::create_client<FollowJointTrajectory>(
    node, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
  if (!action->wait_for_action_server(SERVICE_TIMEOUT)) {
    RCLCPP_FATAL(node->get_logger(), "Action server timeout");
    std::exit(1);
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = UR_JOINT_NAMES;
  trajectory_msgs::msg::JointTrajectoryPoint start, end;
  start.positions = current;
  end.positions = target;
  end.time_from_start = rclcpp::Duration::from_seconds(duration);
  goal.trajectory.points = {start, end};

  RCLCPP_INFO(node->get_logger(), "Moving to equilibrium [%s]...",
    fmt::format("{:.2f}", fmt::join(target, ", ")).c_str());

  auto goal_handle = action->async_send_goal(goal);
  executor.spin_until_future_complete(goal_handle);
  if (!goal_handle.get()) {
    RCLCPP_FATAL(node->get_logger(), "Goal rejected");
    std::exit(1);
  }

  auto result = action->async_get_result(goal_handle.get());
  executor.spin_until_future_complete(result);
  if (result.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_FATAL(node->get_logger(), "Movement failed");
    std::exit(1);
  }
  RCLCPP_INFO(node->get_logger(), "Movement complete");
}

void SaveEquilibriumPose(rclcpp::Node::SharedPtr node,
                        const kinematics::KinematicsComponents& kc,
                        const JointVector& target) {
  KDL::ChainFkSolverPos_recursive fk(kc.robot_chain);
  KDL::JntArray q(DOF);
  for (size_t i = 0; i < DOF; ++i) q(i) = target[i];

  KDL::Frame wrist;
  if (fk.JntToCart(q, wrist) < 0) {
    RCLCPP_FATAL(node->get_logger(), "Forward kinematics failed");
    std::exit(1);
  }

  KDL::Frame tool = wrist * kc.tool_offset;
  double x, y, z, w;
  tool.M.GetQuaternion(x, y, z, w);
  logging::LogPose(node->get_logger(), "Equilibrium:",
    Vector3d(tool.p.x(), tool.p.y(), tool.p.z()),
    Eigen::Quaterniond(w, x, y, z));

  YAML::Node config;
  config["admittance_node"]["ros__parameters"]["equilibrium.position"] = 
    std::vector<double>{tool.p.x(), tool.p.y(), tool.p.z()};
  config["admittance_node"]["ros__parameters"]["equilibrium.orientation"] = 
    std::vector<double>{w, x, y, z};

  const char* workspace_env = std::getenv("ROS_WORKSPACE");
  std::string workspace = workspace_env ? workspace_env : 
                         std::string(std::getenv("HOME")) + "/ros2_ws";
  auto path = std::filesystem::path(workspace) / "src" / "ur_admittance_controller" / "config" / "equilibrium.yaml";
  std::ofstream file(path.string());
  if (!file) {
    RCLCPP_FATAL(node->get_logger(), "Failed to save equilibrium: %s", path.string().c_str());
    std::exit(1);
  }
  file << config;
  RCLCPP_INFO(node->get_logger(), "Saved equilibrium to %s", path.string().c_str());
}

}

int main(int argc, char** argv) {
  using namespace ur_admittance_controller;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("init_robot");

  node->declare_parameter("equilibrium.joints",
    JointVector{0.0, -1.571, 1.571, -1.571, -1.571, 0.0});
  node->declare_parameter("movement_duration", DEFAULT_MOVEMENT_DURATION);

  const auto target = node->get_parameter("equilibrium.joints").as_double_array();
  const auto duration = node->get_parameter("movement_duration").as_double();

  RCLCPP_INFO(node->get_logger(), "Initializing UR robot to equilibrium position");

  const auto kinematics = LoadKinematics(node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  JointVector current(DOF);
  bool received = false;
  auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1,
    [&](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
      for (size_t i = 0; i < DOF; ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), UR_JOINT_NAMES[i]);
        if (it != msg->name.end())
          current[i] = msg->position[std::distance(msg->name.begin(), it)];
      }
      received = true;
    });

  RCLCPP_INFO(node->get_logger(), "Waiting for joint states...");
  auto deadline = std::chrono::steady_clock::now() + SERVICE_TIMEOUT;
  while (!received && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  if (!received) {
    RCLCPP_FATAL(node->get_logger(), "Joint states timeout");
    std::exit(1);
  }

  MoveToEquilibrium(node, executor, current, target, duration);
  SaveEquilibriumPose(node, kinematics, target);

  RCLCPP_INFO(node->get_logger(), "Robot initialization complete");
  rclcpp::shutdown();
  return 0;
}