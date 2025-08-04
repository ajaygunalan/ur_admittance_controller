// Initialize robot to equilibrium position and save probe tip pose
#include <chrono>
#include <fstream>
#include <future>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <utilities/types.hpp>
#include <utilities/error.hpp>
#include <utilities/kinematics.hpp>
#include <utilities/logging.hpp>
#include <utilities/constants.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using SwitchController = controller_manager_msgs::srv::SwitchController;

// Constants
static constexpr size_t JOINT_COUNT = 6;
static constexpr auto ROBOT_DESC_TIMEOUT = ur_admittance_controller::constants::DEFAULT_TIMEOUT;
static constexpr auto JOINT_STATE_TIMEOUT = ur_admittance_controller::constants::DEFAULT_TIMEOUT;
static constexpr auto ACTION_SERVER_TIMEOUT = ur_admittance_controller::constants::DEFAULT_TIMEOUT;
static constexpr auto SERVICE_TIMEOUT = ur_admittance_controller::constants::SERVICE_TIMEOUT;

static constexpr std::array<double, JOINT_COUNT> DEFAULT_EQUILIBRIUM = {
  0.0, -1.571, 1.571, -1.571, -1.571, 0.0
};

// Data structures
struct KinematicSolver {
  KDL::Chain kdl_chain;  // Store chain to avoid dangling reference
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  KDL::Frame tool_offset;
};

// Get robot_description from parameter server following ROS2 best practice
// @param node ROS2 node for getting parameter
// @return URDF string from robot_description parameter
// @throws std::runtime_error if parameter not set
std::string getRobotDescription(rclcpp::Node::SharedPtr node) {
  // Helper for consistent error handling (follows ROS2 patterns)
  auto throwError = [&node](const std::string& msg) {
    RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
    throw std::runtime_error(msg);
  };

  // Create parameter client to get parameter from robot_state_publisher node
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      node, "/robot_state_publisher");

  // Wait for the service to be available
  if (!parameters_client->wait_for_service(ROBOT_DESC_TIMEOUT)) {
    throwError("robot_state_publisher service not available. "
               "Is robot_state_publisher running?");
  }

  // Get the robot_description parameter
  auto parameters = parameters_client->get_parameters({"robot_description"});

  if (parameters.empty() || parameters[0].as_string().empty()) {
    throwError("robot_description parameter not found or empty on robot_state_publisher node");
  }

  auto robot_description = parameters[0].as_string();
  RCLCPP_INFO(node->get_logger(),
              "Got robot description from robot_state_publisher (%lu bytes)",
              robot_description.size());

  return robot_description;
}

// Parse URDF string to create kinematic solver
// @param urdf URDF robot description string
// @param logger ROS logger for error messages
// @return KinematicSolver with chain and FK solver initialized
// @throws std::runtime_error if parsing fails
KinematicSolver parseUrdfToKinematicSolver(
    const std::string& urdf, rclcpp::Logger logger) {
  KinematicSolver solver;

  // Use shared kinematics utility
  auto result = ur_admittance_controller::kinematics::InitializeFromUrdfString(
      urdf, "base_link", "p42v_link1");

  if (!result) {
    auto msg = result.error().message;
    RCLCPP_ERROR(logger, "%s", msg.c_str());
    throw std::runtime_error(msg);
  }

  auto& components = result.value();
  solver.kdl_chain = components.robot_chain;
  solver.tool_offset = components.tool_offset;

  // Create FK solver (following ROS2 pattern - callers create their own solvers)
  solver.fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(solver.kdl_chain);

  RCLCPP_INFO(logger, "KDL kinematics initialized with %zu joints", components.num_joints);
  ur_admittance_controller::logging::LogVector3(logger, "Tool offset from wrist_3_link to p42v_link1 (probe tip):",
                      ur_admittance_controller::Vector3d(solver.tool_offset.p.x(), solver.tool_offset.p.y(), solver.tool_offset.p.z()));

  return solver;
}

std::vector<double> waitForJointStates(rclcpp::Node::SharedPtr node,
                                       const std::vector<std::string>& joint_names,
                                       rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) {
  // Create promise/future for efficient waiting
  std::promise<std::vector<double>> positions_promise;
  auto positions_future = positions_promise.get_future();

  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", ur_admittance_controller::constants::DEFAULT_QUEUE_SIZE,
    [&positions_promise, &joint_names, node](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::vector<double> current_positions(JOINT_COUNT);

      // Extract positions for our joints
      for (size_t i = 0; i < joint_names.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
        if (it == msg->name.end()) return;  // Joint not found

        size_t idx = std::distance(msg->name.begin(), it);
        if (idx >= msg->position.size()) return;  // Position not available

        current_positions[i] = msg->position[idx];
      }

      RCLCPP_INFO(node->get_logger(), "Joint positions received");
      positions_promise.set_value(current_positions);
    });

  auto status = executor->spin_until_future_complete(positions_future, JOINT_STATE_TIMEOUT);

  // Tier 2: Setup failure - throw on timeout
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    auto msg = "Timeout waiting for joint states";
    RCLCPP_ERROR(node->get_logger(), "%s", msg);
    throw std::runtime_error(msg);
  }

  return positions_future.get();
}

bool executeTrajectoryAndWait(rclcpp::Node::SharedPtr node,
                             rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
                             const FollowJointTrajectory::Goal& goal,
                             rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) {
  RCLCPP_INFO(node->get_logger(), "Sending trajectory...");

  auto goal_handle_future = client->async_send_goal(goal);
  if (executor->spin_until_future_complete(goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected by server");
    return false;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (executor->spin_until_future_complete(result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get result");
    return false;
  }

  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed");
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Trajectory completed successfully");
  return true;
}

void computeToolPoseAndSaveYaml(const KinematicSolver& solver,
                               const std::vector<double>& joint_positions,
                               rclcpp::Logger logger) {
  KDL::JntArray q(JOINT_COUNT);
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    q(i) = joint_positions[i];
  }

  KDL::Frame wrist_frame;
  if (solver.fk_solver->JntToCart(q, wrist_frame) < 0) {
    throw std::runtime_error("Forward kinematics computation failed");
  }

  KDL::Frame tool_frame = wrist_frame * solver.tool_offset;

  std::vector<double> position = {tool_frame.p.x(), tool_frame.p.y(), tool_frame.p.z()};

  double x, y, z, w;
  tool_frame.M.GetQuaternion(x, y, z, w);
  std::vector<double> orientation = {w, x, y, z};

  ur_admittance_controller::logging::LogPose(logger, "Tool pose:", 
                   ur_admittance_controller::Vector3d(position[0], position[1], position[2]),
                   Eigen::Quaterniond(orientation[0], orientation[1], orientation[2], orientation[3]));

  const char* ros_workspace = std::getenv("ROS_WORKSPACE");
  std::string workspace = ros_workspace ? ros_workspace :
                         (std::string(std::getenv("HOME")) + "/ros2_ws");
  std::string equilibrium_file = workspace + "/src/ur_admittance_controller/config/equilibrium.yaml";

  std::ofstream fout(equilibrium_file);
  if (!fout.is_open()) {
    throw std::runtime_error("Failed to open " + equilibrium_file + " for writing");
  }

  fout << "admittance_node:\n"
       << "  ros__parameters:\n"
       << "    equilibrium.position: [" << position[0] << ", " << position[1] << ", " << position[2] << "]\n"
       << "    equilibrium.orientation: [" << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << ", " << orientation[3] << "]\n";

  RCLCPP_INFO(logger, "Saved equilibrium pose to %s", equilibrium_file.c_str());
}

void switchToVelocityController(rclcpp::Node::SharedPtr node) {
  auto client = node->create_client<SwitchController>("/controller_manager/switch_controller");

  if (!client->wait_for_service(SERVICE_TIMEOUT)) {
    RCLCPP_ERROR(node->get_logger(), "Switch controller service not available");
    return;
  }

  auto request = std::make_shared<SwitchController::Request>();
  request->deactivate_controllers = {"scaled_joint_trajectory_controller"};
  request->activate_controllers = {"forward_velocity_controller"};
  request->strictness = SwitchController::Request::BEST_EFFORT;

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, SERVICE_TIMEOUT) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to switch controller");
    return;
  }

  auto response = future.get();
  if (!response->ok) {
    RCLCPP_ERROR(node->get_logger(), "Controller switch failed");
    return;
  }

  RCLCPP_INFO(node->get_logger(), "Switched to velocity controller");
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<rclcpp::Node>("equilibrium_initializer");

    std::vector<std::string> joint_names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    // Get robot description before creating executor
    std::string robot_description = getRobotDescription(node);

    // Now create executor and add node for later use
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);

    node->declare_parameter("equilibrium.joints",
      std::vector<double>(DEFAULT_EQUILIBRIUM.begin(), DEFAULT_EQUILIBRIUM.end()));
    node->declare_parameter("movement_duration", ur_admittance_controller::constants::DEFAULT_MOVEMENT_DURATION);

    auto equilibrium_positions = node->get_parameter("equilibrium.joints").as_double_array();
    auto movement_duration = node->get_parameter("movement_duration").as_double();

    if (equilibrium_positions.size() != JOINT_COUNT) {
      throw std::runtime_error("equilibrium.joints must have 6 values");
    }

    ur_admittance_controller::logging::LogJoints(node->get_logger(), "Equilibrium:", equilibrium_positions);

    auto solver = parseUrdfToKinematicSolver(robot_description, node->get_logger());
    auto current_positions = waitForJointStates(node, joint_names, executor);

    FollowJointTrajectory::Goal trajectory_goal;
    trajectory_goal.trajectory.joint_names = joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    start_point.positions = current_positions;
    start_point.velocities.resize(joint_names.size(), 0.0);
    start_point.time_from_start = rclcpp::Duration(0, 0);
    trajectory_goal.trajectory.points.push_back(start_point);

    trajectory_msgs::msg::JointTrajectoryPoint end_point;
    end_point.positions = equilibrium_positions;
    end_point.velocities.resize(joint_names.size(), 0.0);
    end_point.time_from_start = rclcpp::Duration::from_seconds(movement_duration);
    trajectory_goal.trajectory.points.push_back(end_point);

    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      node, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

    if (!action_client->wait_for_action_server(ACTION_SERVER_TIMEOUT)) {
      RCLCPP_ERROR(node->get_logger(), "Action server not available");
      return 1;
    }

    if (!executeTrajectoryAndWait(node, action_client, trajectory_goal, executor)) {
      return 1;
    }

    computeToolPoseAndSaveYaml(solver, equilibrium_positions, node->get_logger());

    // switchToVelocityController(node);

    rclcpp::shutdown();
    return 0;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Initialization failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
