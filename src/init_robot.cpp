/**
 * @file init_robot.cpp
 * @brief Initialize robot to equilibrium position using trajectory controller.
 *        Computes forward kinematics to p42v_link1 (probe tip) and saves
 *        the Cartesian equilibrium pose for admittance control.
 */

#include <chrono>
#include <fstream>
#include <future>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using SwitchController = controller_manager_msgs::srv::SwitchController;

// Constants
static constexpr size_t JOINT_COUNT = 6;
static constexpr auto ROBOT_DESC_TIMEOUT = std::chrono::seconds(10);
static constexpr auto JOINT_STATE_TIMEOUT = std::chrono::seconds(10);
static constexpr auto ACTION_SERVER_TIMEOUT = std::chrono::seconds(10);
static constexpr auto SERVICE_TIMEOUT = std::chrono::seconds(5);
static constexpr double DEFAULT_MOVEMENT_DURATION = 12.0;

// Default equilibrium positions (radians)
static constexpr std::array<double, JOINT_COUNT> DEFAULT_EQUILIBRIUM = {
  0.0, -1.571, 1.571, -1.571, -1.571, 0.0
};

// Data structures
struct KinematicSolver {
  KDL::Chain kdl_chain;  // Store chain to avoid dangling reference
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  KDL::Frame tool_offset;
};

// Helper templates for error handling
template<typename Logger>
inline void checkFuture(rclcpp::FutureReturnCode status, Logger logger, const std::string& msg) {
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s", msg.c_str());
    throw std::runtime_error(msg);
  }
}

template<typename Logger>
inline void checkCondition(bool condition, Logger logger, const std::string& msg) {
  if (!condition) {
    RCLCPP_ERROR(logger, "%s", msg.c_str());
    throw std::runtime_error(msg);
  }
}

std::string waitForRobotDescription(rclcpp::Node::SharedPtr node,
                                   rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) {
  // Create promise/future for efficient waiting
  std::promise<std::string> description_promise;
  auto description_future = description_promise.get_future();
  
  // Create subscription with TRANSIENT_LOCAL QoS to match robot_state_publisher
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                .reliable()
                .transient_local();
  
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "/robot_description", qos,
    [&description_promise, node](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(node->get_logger(), 
                  "Received robot description from topic (%lu bytes)", 
                  msg->data.size());
      description_promise.set_value(msg->data);
    });
  
  RCLCPP_INFO(node->get_logger(), "Waiting for robot_description topic...");
  
  // Use the provided executor instead of creating a new one
  auto status = executor->spin_until_future_complete(description_future, ROBOT_DESC_TIMEOUT);
  
  checkFuture(status, node->get_logger(), 
              "Timeout waiting for robot_description topic. Is robot_state_publisher running?");
  
  return description_future.get();
}

KinematicSolver parseUrdfToKinematicSolver(const std::string& urdf, rclcpp::Logger logger) {
  KinematicSolver solver;
  
  // Parse URDF to KDL tree
  KDL::Tree kdl_tree;
  checkCondition(kdl_parser::treeFromString(urdf, kdl_tree), logger, 
                 "Failed to parse URDF to KDL tree");
  
  // Extract main chain
  checkCondition(kdl_tree.getChain("base_link", "wrist_3_link", solver.kdl_chain), logger,
                 "Failed to extract kinematic chain from base_link to wrist_3_link");
  
  // Extract tool transform inline
  KDL::Chain tool_chain;
  if (kdl_tree.getChain("wrist_3_link", "p42v_link1", tool_chain)) {
    KDL::ChainFkSolverPos_recursive tool_fk(tool_chain);
    KDL::JntArray zero_joint(tool_chain.getNrOfJoints());
    tool_fk.JntToCart(zero_joint, solver.tool_offset);
    RCLCPP_INFO(logger, "Tool offset from wrist_3_link to p42v_link1 (probe tip): [%.3f, %.3f, %.3f]",
                solver.tool_offset.p.x(), solver.tool_offset.p.y(), solver.tool_offset.p.z());
  } else {
    RCLCPP_WARN(logger, "p42v_link1 not found, using identity transform");
    solver.tool_offset = KDL::Frame::Identity();
  }
  
  // Create FK solver
  solver.fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(solver.kdl_chain);
  RCLCPP_INFO(logger, "KDL kinematics initialized with %d joints", solver.kdl_chain.getNrOfJoints());
  
  return solver;
}

std::vector<double> waitForJointStates(rclcpp::Node::SharedPtr node,
                                       const std::vector<std::string>& joint_names,
                                       rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) {
  // Create promise/future for efficient waiting
  std::promise<std::vector<double>> positions_promise;
  auto positions_future = positions_promise.get_future();
  
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
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
  
  // Use the provided executor instead of creating a new one
  auto status = executor->spin_until_future_complete(positions_future, JOINT_STATE_TIMEOUT);
  
  checkFuture(status, node->get_logger(), "Timeout waiting for joint states");
  
  return positions_future.get();
}

bool executeTrajectoryAndWait(rclcpp::Node::SharedPtr node,
                             rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
                             const FollowJointTrajectory::Goal& goal,
                             rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) {
  RCLCPP_INFO(node->get_logger(), "Sending trajectory...");
  auto goal_handle_future = client->async_send_goal(goal);
  
  if (executor->spin_until_future_complete(goal_handle_future) != 
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    return false;
  }
  
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected by server");
    return false;
  }
  
  // Wait for result
  auto result_future = client->async_get_result(goal_handle);
  if (executor->spin_until_future_complete(result_future) != 
      rclcpp::FutureReturnCode::SUCCESS) {
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
  
  // Convert to KDL
  KDL::JntArray q(JOINT_COUNT);
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    q(i) = joint_positions[i];
  }
  
  // Compute FK
  KDL::Frame wrist_frame;
  checkCondition(solver.fk_solver->JntToCart(q, wrist_frame) >= 0, logger,
                 "Forward kinematics computation failed");
  
  // Apply tool offset to get p42v_link1 (probe tip) frame
  KDL::Frame tool_frame = wrist_frame * solver.tool_offset;
  
  // Extract pose at probe tip
  std::vector<double> position = {
    tool_frame.p.x(), 
    tool_frame.p.y(), 
    tool_frame.p.z()
  };
  
  double x, y, z, w;
  tool_frame.M.GetQuaternion(x, y, z, w);
  std::vector<double> orientation = {w, x, y, z};
  
  RCLCPP_INFO(logger, "Tool pose - Position: [%.3f, %.3f, %.3f], Orientation: [%.3f, %.3f, %.3f, %.3f]",
              position[0], position[1], position[2],
              orientation[0], orientation[1], orientation[2], orientation[3]);
  
  // Save to YAML
  std::ofstream fout("src/ur_admittance_controller/config/equilibrium.yaml");
  checkCondition(fout.is_open(), logger, "Failed to open equilibrium.yaml for writing");
  
  fout << "admittance_node:\n"
       << "  ros__parameters:\n"
       << "    equilibrium.position: [" << position[0] << ", " << position[1] << ", " << position[2] << "]\n"
       << "    equilibrium.orientation: [" << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << ", " << orientation[3] << "]\n";
  
  RCLCPP_INFO(logger, "Saved equilibrium pose to YAML");
  RCLCPP_INFO(logger, "Run: ros2 run ur_admittance_controller admittance_node "
                      "--ros-args --params-file "
                      "src/ur_admittance_controller/config/equilibrium.yaml");
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
  if (rclcpp::spin_until_future_complete(node, future, SERVICE_TIMEOUT) != 
      rclcpp::FutureReturnCode::SUCCESS) {
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

// Main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<rclcpp::Node>("equilibrium_initializer");
    
    // Create a single executor to reuse throughout
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    
    // Get robot configuration inline
    std::vector<std::string> joint_names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    
    // Get robot_description from topic
    std::string robot_description = waitForRobotDescription(node, executor);
    
    // Declare and get equilibrium parameters
    node->declare_parameter("equilibrium.joints", 
      std::vector<double>(DEFAULT_EQUILIBRIUM.begin(), DEFAULT_EQUILIBRIUM.end()));
    node->declare_parameter("movement_duration", DEFAULT_MOVEMENT_DURATION);
    
    std::vector<double> equilibrium_positions = node->get_parameter("equilibrium.joints").as_double_array();
    double movement_duration = node->get_parameter("movement_duration").as_double();
    
    // Validate
    checkCondition(equilibrium_positions.size() == JOINT_COUNT, node->get_logger(),
                   "Invalid configuration: equilibrium.joints must have 6 values");
    
    RCLCPP_INFO(node->get_logger(), "Joint space equilibrium: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                equilibrium_positions[0], equilibrium_positions[1], 
                equilibrium_positions[2], equilibrium_positions[3], 
                equilibrium_positions[4], equilibrium_positions[5]);
    
    // Parse URDF and build kinematic solver
    auto solver = parseUrdfToKinematicSolver(robot_description, node->get_logger());
    
    // Wait for current joint states
    auto current_positions = waitForJointStates(node, joint_names, executor);
    
    // Build trajectory message inline
    FollowJointTrajectory::Goal trajectory_goal;
    trajectory_goal.trajectory.joint_names = joint_names;
    
    // Start point
    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    start_point.positions = current_positions;
    start_point.velocities.resize(joint_names.size(), 0.0);
    start_point.time_from_start = rclcpp::Duration(0, 0);
    trajectory_goal.trajectory.points.push_back(start_point);
    
    // End point
    trajectory_msgs::msg::JointTrajectoryPoint end_point;
    end_point.positions = equilibrium_positions;
    end_point.velocities.resize(joint_names.size(), 0.0);
    end_point.time_from_start = rclcpp::Duration::from_seconds(movement_duration);
    trajectory_goal.trajectory.points.push_back(end_point);
    
    // Create action client
    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      node, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    
    if (!action_client->wait_for_action_server(ACTION_SERVER_TIMEOUT)) {
      RCLCPP_ERROR(node->get_logger(), "Action server not available");
      return 1;
    }
    
    // Execute trajectory
    if (!executeTrajectoryAndWait(node, action_client, trajectory_goal, executor)) {
      return 1;
    }
    
    // Compute tool pose and save to YAML
    computeToolPoseAndSaveYaml(solver, equilibrium_positions, 
                              node->get_logger());
    
    // Controller switch (commented as in original)
    // switchToVelocityController(node);
    
    rclcpp::shutdown();
    return 0;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), 
                 "Initialization failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
