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
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include <string>

class EquilibriumInitializer : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  EquilibriumInitializer() : Node("equilibrium_initializer"),
    joint_names_({"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}) {
    // Declare parameters
    this->declare_parameter("equilibrium.position", std::vector<double>{0.49, 0.13, 0.49});
    this->declare_parameter("equilibrium.orientation", std::vector<double>{-0.00, -0.71, 0.71, 0.00});
    this->declare_parameter("movement_duration", 12.0);
    movement_duration_ = this->get_parameter("movement_duration").as_double();
    
    // Setup kinematics and compute equilibrium
    if (!setupKinematics() || !computeEquilibriumJoints())
      throw std::runtime_error("Initialization failed");
    // Create subscription and clients
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&EquilibriumInitializer::jointStateCallback, this, std::placeholders::_1));
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
    while (!current_positions_received_) {
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Execute initialization sequence
    executeInitialization();
  }

private:
  bool setupKinematics() {
    // Get robot_description from robot_state_publisher
    auto param_client = this->create_client<rcl_interfaces::srv::GetParameters>(
      "/robot_state_publisher/get_parameters");
    if (!param_client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "robot_state_publisher parameter service not available");
      return false;
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");
    auto future = param_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
        std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to get robot_description");
      return false;
    }
    
    auto response = future.get();
    if (response->values.empty() || response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      RCLCPP_ERROR(get_logger(), "Invalid robot_description parameter");
      return false;
    }
    
    std::string urdf_string = response->values[0].string_value;
    
    // Parse URDF into KDL tree
    if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF into KDL tree");
      return false;
    }
    
    // Always extract chain from base_link to wrist_3_link (6 movable joints)
    if (!kdl_tree_.getChain("base_link", "wrist_3_link", kdl_chain_)) {
      RCLCPP_ERROR(get_logger(), "Failed to extract kinematic chain");
      return false;
    }
    
    // Get the fixed transform from wrist_3_link to tool_payload
    KDL::Chain tool_chain;
    if (!kdl_tree_.getChain("wrist_3_link", "tool_payload", tool_chain)) {
      RCLCPP_ERROR(get_logger(), "Failed to extract tool_payload chain");
      return false;
    }
    
    // Get the fixed transform from wrist_3 to tool_payload
    KDL::ChainFkSolverPos_recursive tool_fk(tool_chain);
    KDL::JntArray zero_joint(tool_chain.getNrOfJoints());
    tool_fk.JntToCart(zero_joint, ft_offset_);
    
    RCLCPP_INFO(get_logger(), "Tool offset from wrist_3_link: [%.3f, %.3f, %.3f]",
                ft_offset_.p.x(), ft_offset_.p.y(), ft_offset_.p.z());
    
    // Create FK and IK solvers
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
    
    RCLCPP_INFO(get_logger(), "KDL kinematics initialized with %d joints", kdl_chain_.getNrOfJoints());
    return true;
  }
  
  bool computeEquilibriumJoints() {
    // Get equilibrium parameters
    auto eq_pos = this->get_parameter("equilibrium.position").as_double_array();
    auto eq_ori = this->get_parameter("equilibrium.orientation").as_double_array();
    
    if (eq_pos.size() != 3 || eq_ori.size() != 4) {
      RCLCPP_ERROR(get_logger(), "Invalid equilibrium parameters");
      return false;
    }
    
    // Create target frame for tool from position and quaternion (wxyz format)
    KDL::Frame target_tool_frame(KDL::Rotation::Quaternion(eq_ori[1], eq_ori[2], eq_ori[3], eq_ori[0]),
                                 KDL::Vector(eq_pos[0], eq_pos[1], eq_pos[2]));
    
    // Convert target from tool frame to wrist_3 frame for IK
    KDL::Frame target_wrist3_frame = target_tool_frame * ft_offset_.Inverse();
    
    // Initial joint positions for IK seed (6 joints)
    KDL::JntArray q_init(6);
    double init_vals[] = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
    for (int i = 0; i < 6; ++i) q_init(i) = init_vals[i];
    
    // Solve IK for wrist_3 position
    KDL::JntArray q_out(6);
    int ik_result = ik_solver_->CartToJnt(q_init, target_wrist3_frame, q_out);
    
    if (ik_result < 0) {
      RCLCPP_ERROR(get_logger(), "IK failed with error code %d", ik_result);
      return false;
    }
    
    // Store equilibrium positions (6 joints)
    equilibrium_positions_.resize(6);
    for (unsigned int i = 0; i < 6; ++i) equilibrium_positions_[i] = q_out(i);
    
    RCLCPP_INFO(get_logger(), "IK solved: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                equilibrium_positions_[0], equilibrium_positions_[1], equilibrium_positions_[2],
                equilibrium_positions_[3], equilibrium_positions_[4], equilibrium_positions_[5]);
    
    // Verify with FK
    KDL::Frame fk_wrist3_frame;
    fk_solver_->JntToCart(q_out, fk_wrist3_frame);
    
    // Apply fixed transform to get tool position
    KDL::Frame fk_tool_frame = fk_wrist3_frame * ft_offset_;
    
    RCLCPP_INFO(get_logger(), "FK verification - tool at: [%.3f, %.3f, %.3f]",
                fk_tool_frame.p.x(), fk_tool_frame.p.y(), fk_tool_frame.p.z());
    RCLCPP_INFO(get_logger(), "Target was: [%.3f, %.3f, %.3f]", 
                eq_pos[0], eq_pos[1], eq_pos[2]);
    
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
    if (!trajectory_client_) return false;
    
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
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to send trajectory goal");
      return false;
    }
    
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Trajectory goal was rejected");
      return false;
    }
    // Wait for result
    auto result_future = trajectory_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to get trajectory result");
      return false;
    }
    
    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "Trajectory execution failed");
      return false;
    }
    
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
        std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to switch controllers");
      return false;
    }
    
    auto response = future.get();
    if (!response->ok) {
      RCLCPP_ERROR(get_logger(), "Controller switch rejected");
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "Successfully switched to velocity controller");
    return true;
  }
  
  void executeInitialization() {
    if (moveToEquilibrium()) {
      // Small delay before switching controllers
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      if (switchToVelocityController()) {
        RCLCPP_INFO(get_logger(), "Robot initialized successfully");
        // Delay before shutting down to ensure controller switch completes
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
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
  KDL::Frame ft_offset_;  // Fixed transform from wrist_3 to tool_payload
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<EquilibriumInitializer>();
    // Run for a bit to complete initialization, then shutdown
    rclcpp::spin_some(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}