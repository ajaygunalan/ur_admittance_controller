/**
 * @file init_robot.cpp
 * @brief Initialize robot to equilibrium position using trajectory controller,
 *        then switch to velocity controller for admittance control.
 * 
 * C++ implementation of init_robot.py using KDL for kinematics
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
// KDL headers
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

class EquilibriumInitializer : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  EquilibriumInitializer() : Node("equilibrium_initializer"),
    joint_names_({"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}) {
    // Declare parameters - now using joint space equilibrium
    // Good working pose: shoulder at -90°, elbow bent ~90°, wrist oriented down
    this->declare_parameter("equilibrium.joints", std::vector<double>{
      0.0,     // shoulder_pan: facing forward
      -1.571,  // shoulder_lift: -90° (horizontal)
      1.571,   // elbow: ~90° bend
      -1.571,  // wrist_1: -90° 
      -1.571,  // wrist_2: ~-90°
      0.0      // wrist_3: 0°
    });
    this->declare_parameter("movement_duration", 12.0);
    movement_duration_ = this->get_parameter("movement_duration").as_double();
    
    // Setup kinematics and compute equilibrium
    if (!setupKinematics() || !computeEquilibriumJoints())
      throw std::runtime_error("Initialization failed");
    // Create subscription and clients
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
    
    // Wait for services
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10)))
      throw std::runtime_error("Trajectory action server not available");
    if (!switch_controller_client_->wait_for_service(std::chrono::seconds(5)))
      throw std::runtime_error("Switch controller service not available");
    // Wait for initial joint states
    while (!current_positions_received_ && rclcpp::ok()) {
      rclcpp::spin_some(get_node_base_interface());
    }
    // Execute initialization sequence
    executeInitialization();
  }

private:
  bool setupKinematics() {
    // Get robot_description from robot_state_publisher
    auto param_client = this->create_client<rcl_interfaces::srv::GetParameters>(
      "/robot_state_publisher/get_parameters");
    if (!param_client->wait_for_service(std::chrono::seconds(5)))
      return false;
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");
    auto future = param_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
        std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS)
      return false;
    
    auto response = future.get();
    if (response->values.empty() || response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
      return false;
    
    std::string urdf_string = response->values[0].string_value;
    
    // Parse URDF into KDL tree
    if (!kdl_parser::treeFromString(urdf_string, kdl_tree_))
      return false;
    
    // Always extract chain from base_link to wrist_3_link (6 movable joints)
    if (!kdl_tree_.getChain("base_link", "wrist_3_link", kdl_chain_))
      return false;
    
    // Get the fixed transform from wrist_3_link to p42v_link1 (probe tip)
    KDL::Chain tool_chain;
    if (!kdl_tree_.getChain("wrist_3_link", "p42v_link1", tool_chain))
      return false;
    KDL::ChainFkSolverPos_recursive tool_fk(tool_chain);
    KDL::JntArray zero_joint(tool_chain.getNrOfJoints());
    tool_fk.JntToCart(zero_joint, ft_offset_);
    
    RCLCPP_INFO(get_logger(), "Tool offset from wrist_3_link: [%.3f, %.3f, %.3f]",
                ft_offset_.p.x(), ft_offset_.p.y(), ft_offset_.p.z());
    
    // Create FK solver (no IK needed for joint space approach)
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    
    RCLCPP_INFO(get_logger(), "KDL kinematics initialized with %d joints", kdl_chain_.getNrOfJoints());
    return true;
  }
  
  bool computeEquilibriumJoints() {
    // Get equilibrium joint parameters
    auto eq_joints = this->get_parameter("equilibrium.joints").as_double_array();
    
    if (eq_joints.size() != 6) {
      RCLCPP_ERROR(get_logger(), "equilibrium.joints must have 6 values");
      return false;
    }
    
    // Store equilibrium positions directly from parameter
    equilibrium_positions_ = eq_joints;
    
    RCLCPP_INFO(get_logger(), "Joint space equilibrium: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                equilibrium_positions_[0], equilibrium_positions_[1], equilibrium_positions_[2],
                equilibrium_positions_[3], equilibrium_positions_[4], equilibrium_positions_[5]);
    
    // Compute FK to verify tool position
    KDL::JntArray q_eq(6);
    for (size_t i = 0; i < 6; ++i) q_eq(i) = equilibrium_positions_[i];
    
    KDL::Frame fk_wrist3_frame;
    fk_solver_->JntToCart(q_eq, fk_wrist3_frame);
    
    // Apply fixed transform to get tool position
    KDL::Frame fk_tool_frame = fk_wrist3_frame * ft_offset_;
    
    RCLCPP_INFO(get_logger(), "FK computed - tool at: [%.3f, %.3f, %.3f]",
                fk_tool_frame.p.x(), fk_tool_frame.p.y(), fk_tool_frame.p.z());
    
    // Extract quaternion for logging
    double x, y, z, w;
    fk_tool_frame.M.GetQuaternion(x, y, z, w);
    RCLCPP_INFO(get_logger(), "Tool orientation (wxyz): [%.3f, %.3f, %.3f, %.3f]", w, x, y, z);
    
    return true;
  }
  
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (current_positions_received_) return;
    
    // Extract joint positions in order
    current_positions_.resize(joint_names_.size());
    bool all_found = true;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it != msg->name.end() && static_cast<size_t>(std::distance(msg->name.begin(), it)) < msg->position.size()) {
        current_positions_[i] = msg->position[std::distance(msg->name.begin(), it)];
      } else {
        all_found = false;
        break;
      }
    }
    
    if (all_found) {
      current_positions_received_ = true;
      RCLCPP_INFO_ONCE(get_logger(), "Initial joint positions received");
    }
  }
  
  bool moveToEquilibrium() {
    // Create trajectory message
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.joint_names = joint_names_;
    
    // Start point (current position)
    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    start_point.positions = current_positions_;
    start_point.velocities.resize(joint_names_.size(), 0.0);
    start_point.time_from_start = rclcpp::Duration(0, 0);
    trajectory_msg.points.push_back(start_point);
    // End point (equilibrium position)
    trajectory_msgs::msg::JointTrajectoryPoint end_point;
    end_point.positions = equilibrium_positions_;
    end_point.velocities.resize(joint_names_.size(), 0.0);
    end_point.time_from_start = rclcpp::Duration::from_seconds(movement_duration_);
    trajectory_msg.points.push_back(end_point);
    
    // Create goal
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory = trajectory_msg;
    
    // Send goal
    RCLCPP_INFO(get_logger(), "Sending trajectory to equilibrium position...");
    auto goal_handle_future = trajectory_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
      return false;
    
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
      return false;
    // Wait for result
    auto result_future = trajectory_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
      return false;
    
    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
      return false;
    
    RCLCPP_INFO(get_logger(), "Successfully moved to equilibrium position");
    return true;
  }
  
  bool switchToVelocityController() {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->deactivate_controllers = {"scaled_joint_trajectory_controller"};
    request->activate_controllers = {"forward_velocity_controller"};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    
    auto future = switch_controller_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
        std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
      return false;
    
    auto response = future.get();
    if (!response->ok)
      return false;
    
    RCLCPP_INFO(get_logger(), "Successfully switched to velocity controller");
    return true;
  }
  
  void computeForwardKinematics(const std::vector<double>& joint_positions, 
                               std::vector<double>& position, 
                               std::vector<double>& orientation) {
    // Compute FK for given joint positions
    KDL::JntArray q_joints(6);
    for (size_t i = 0; i < 6; ++i) q_joints(i) = joint_positions[i];
    
    KDL::Frame fk_wrist3_frame;
    fk_solver_->JntToCart(q_joints, fk_wrist3_frame);
    
    // Apply fixed transform to get tool position
    KDL::Frame fk_tool_frame = fk_wrist3_frame * ft_offset_;
    
    // Extract position
    position.clear();
    position.push_back(fk_tool_frame.p.x());
    position.push_back(fk_tool_frame.p.y());
    position.push_back(fk_tool_frame.p.z());
    
    // Extract orientation (quaternion WXYZ)
    double x, y, z, w;
    fk_tool_frame.M.GetQuaternion(x, y, z, w);
    orientation.clear();
    orientation.push_back(w);
    orientation.push_back(x);
    orientation.push_back(y);
    orientation.push_back(z);
  }
  
  bool saveEquilibriumToConfig(const std::vector<double>& position, const std::vector<double>& orientation) {
    try {
      // Write to separate equilibrium.yaml for runtime parameters
      std::string config_path = "src/ur_admittance_controller/config/equilibrium.yaml";
      
      // Create clean structure for runtime parameters
      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "admittance_node";
      out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "ros__parameters";
      out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "equilibrium.position" << YAML::Value << YAML::Flow << position;
      out << YAML::Key << "equilibrium.orientation" << YAML::Value << YAML::Flow << orientation;
      out << YAML::EndMap; // ros__parameters
      out << YAML::EndMap; // admittance_node
      out << YAML::EndMap; // root
      
      // Write to file
      std::ofstream fout(config_path);
      fout << out.c_str();
      fout.close();
      
      RCLCPP_INFO(get_logger(), "Saved equilibrium to equilibrium.yaml: pos=[%.3f, %.3f, %.3f], ori=[%.3f, %.3f, %.3f, %.3f]",
                  position[0], position[1], position[2],
                  orientation[0], orientation[1], orientation[2], orientation[3]);
      RCLCPP_INFO(get_logger(), "Run admittance_node with: ros2 run ur_admittance_controller admittance_node --ros-args --params-file src/ur_admittance_controller/config/equilibrium.yaml");
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to save equilibrium to config: %s", e.what());
      return false;
    }
  }
  
  void executeInitialization() {
    if (moveToEquilibrium()) {
      RCLCPP_INFO(get_logger(), "Robot moved to equilibrium position");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      
      // Compute FK for equilibrium joint positions
      std::vector<double> cart_position, cart_orientation;
      computeForwardKinematics(equilibrium_positions_, cart_position, cart_orientation);
      
      // Save to config file
      saveEquilibriumToConfig(cart_position, cart_orientation);
      
      // Controller switching is now done manually after calibration
      // switchToVelocityController();
    }
  }
  
  // Member variables
  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_, equilibrium_positions_;
  double movement_duration_;
  bool current_positions_received_ = false;
  // KDL structures
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::Frame ft_offset_;  // Fixed transform from wrist_3 to p42v_link1 (probe tip)
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<EquilibriumInitializer>();
  rclcpp::spin_some(node);
  
  rclcpp::shutdown();
  return 0;
}