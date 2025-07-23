/**
 * @file test_ik.cpp
 * @brief Test IK by computing joint angles from equilibrium Cartesian pose and moving robot
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

class IKTester : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  
  IKTester() : Node("ik_tester"),
    joint_names_({"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}) {
    
    if (!setupKinematics()) {
      throw std::runtime_error("Failed to setup kinematics");
    }
    
    if (!loadCartesianPose()) {
      throw std::runtime_error("Failed to load Cartesian pose from config");
    }
    
    if (!computeIK()) {
      throw std::runtime_error("IK failed - cannot reach target pose");
    }
    
    // Setup ROS interfaces for movement
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });
    
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
      throw std::runtime_error("Trajectory action server not available");
    }
    
    // Wait for initial joint states
    while (!current_positions_received_ && rclcpp::ok()) {
      rclcpp::spin_some(get_node_base_interface());
    }
    
    // Move robot to IK solution
    moveToIKSolution();
  }

private:
  bool setupKinematics() {
    // Get robot_description from robot_state_publisher
    auto param_client = this->create_client<rcl_interfaces::srv::GetParameters>(
      "/robot_state_publisher/get_parameters");
    if (!param_client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Robot state publisher not available");
      return false;
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");
    auto future = param_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
        std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }
    
    auto response = future.get();
    if (response->values.empty() || response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      return false;
    }
    
    std::string urdf_string = response->values[0].string_value;
    
    // Parse URDF into KDL tree
    if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
      return false;
    }
    
    // Extract chain from base_link to wrist_3_link
    if (!kdl_tree_.getChain("base_link", "wrist_3_link", kdl_chain_)) {
      return false;
    }
    
    // Get the fixed transform from wrist_3_link to p42v_link1
    KDL::Chain tool_chain;
    if (!kdl_tree_.getChain("wrist_3_link", "p42v_link1", tool_chain)) {
      return false;
    }
    KDL::ChainFkSolverPos_recursive tool_fk(tool_chain);
    KDL::JntArray zero_joint(tool_chain.getNrOfJoints());
    tool_fk.JntToCart(zero_joint, tool_offset_);
    
    RCLCPP_INFO(get_logger(), "Tool offset from wrist_3_link: [%.3f, %.3f, %.3f]",
                tool_offset_.p.x(), tool_offset_.p.y(), tool_offset_.p.z());
    
    // Create IK solver
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    
    RCLCPP_INFO(get_logger(), "KDL kinematics initialized with %d joints", kdl_chain_.getNrOfJoints());
    return true;
  }
  
  bool loadCartesianPose() {
    try {
      // Load equilibrium pose from config
      std::string config_path = "src/ur_admittance_controller/config/equilibrium.yaml";
      YAML::Node config = YAML::LoadFile(config_path);
      
      auto pos = config["admittance_node"]["ros__parameters"]["equilibrium.position"].as<std::vector<double>>();
      auto ori = config["admittance_node"]["ros__parameters"]["equilibrium.orientation"].as<std::vector<double>>();
      
      if (pos.size() != 3 || ori.size() != 4) {
        RCLCPP_ERROR(get_logger(), "Invalid pose dimensions");
        return false;
      }
      
      target_position_ = pos;
      target_orientation_ = ori;
      
      RCLCPP_INFO(get_logger(), "Loaded target pose from config:");
      RCLCPP_INFO(get_logger(), "  Position: [%.3f, %.3f, %.3f]", pos[0], pos[1], pos[2]);
      RCLCPP_INFO(get_logger(), "  Orientation (wxyz): [%.3f, %.3f, %.3f, %.3f]", 
                  ori[0], ori[1], ori[2], ori[3]);
      
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load config: %s", e.what());
      return false;
    }
  }
  
  bool computeIK() {
    // Create target frame for tool
    KDL::Frame target_tool_frame(
      KDL::Rotation::Quaternion(target_orientation_[1], target_orientation_[2], 
                                target_orientation_[3], target_orientation_[0]),
      KDL::Vector(target_position_[0], target_position_[1], target_position_[2])
    );
    
    // Convert target from tool frame to wrist_3 frame for IK
    KDL::Frame target_wrist3_frame = target_tool_frame * tool_offset_.Inverse();
    
    RCLCPP_INFO(get_logger(), "Target wrist_3 position: [%.3f, %.3f, %.3f]",
                target_wrist3_frame.p.x(), target_wrist3_frame.p.y(), target_wrist3_frame.p.z());
    
    // Try different initial guesses for IK
    std::vector<std::vector<double>> init_guesses = {
      {0.0, -1.57, 0.0, 0.0, 0.0, 0.0},      // Simple start
      {0.0, -1.57, 1.57, -1.57, -1.57, 0.0},  // Expected solution
      {0.0, -2.0, 2.0, -2.0, -1.57, 0.0},     // Alternative
    };
    
    for (size_t guess_idx = 0; guess_idx < init_guesses.size(); ++guess_idx) {
      KDL::JntArray q_init(6);
      for (int i = 0; i < 6; ++i) {
        q_init(i) = init_guesses[guess_idx][i];
      }
      
      // Solve IK
      KDL::JntArray q_out(6);
      int ik_result = ik_solver_->CartToJnt(q_init, target_wrist3_frame, q_out);
      
      if (ik_result >= 0) {
        RCLCPP_INFO(get_logger(), "\n=== IK SUCCESS with guess %zu ===", guess_idx);
        RCLCPP_INFO(get_logger(), "IK solution: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    q_out(0), q_out(1), q_out(2), q_out(3), q_out(4), q_out(5));
        
        // Store solution
        ik_solution_.resize(6);
        for (int i = 0; i < 6; ++i) {
          ik_solution_[i] = q_out(i);
        }
        
        // Verify with FK
        KDL::Frame fk_wrist3_frame;
        fk_solver_->JntToCart(q_out, fk_wrist3_frame);
        KDL::Frame fk_tool_frame = fk_wrist3_frame * tool_offset_;
        
        RCLCPP_INFO(get_logger(), "FK verification - tool at: [%.3f, %.3f, %.3f]",
                    fk_tool_frame.p.x(), fk_tool_frame.p.y(), fk_tool_frame.p.z());
        
        double pos_error = (fk_tool_frame.p - KDL::Vector(target_position_[0], 
                                                          target_position_[1], 
                                                          target_position_[2])).Norm();
        RCLCPP_INFO(get_logger(), "Position error: %.6f m", pos_error);
        
        return true;
      }
    }
    
    RCLCPP_ERROR(get_logger(), "IK failed with all initial guesses");
    return false;
  }
  
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (current_positions_received_) return;
    
    current_positions_.resize(joint_names_.size());
    bool all_found = true;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it != msg->name.end()) {
        current_positions_[i] = msg->position[std::distance(msg->name.begin(), it)];
      } else {
        all_found = false;
        break;
      }
    }
    
    if (all_found) {
      current_positions_received_ = true;
      RCLCPP_INFO_ONCE(get_logger(), "Current joint positions received");
    }
  }
  
  bool moveToIKSolution() {
    // Create trajectory
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.joint_names = joint_names_;
    
    // Start point (current position)
    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    start_point.positions = current_positions_;
    start_point.velocities.resize(joint_names_.size(), 0.0);
    start_point.time_from_start = rclcpp::Duration(0, 0);
    trajectory_msg.points.push_back(start_point);
    
    // End point (IK solution)
    trajectory_msgs::msg::JointTrajectoryPoint end_point;
    end_point.positions = ik_solution_;
    end_point.velocities.resize(joint_names_.size(), 0.0);
    end_point.time_from_start = rclcpp::Duration::from_seconds(8.0);
    trajectory_msg.points.push_back(end_point);
    
    // Create goal
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory = trajectory_msg;
    
    // Send goal
    RCLCPP_INFO(get_logger(), "Moving robot to IK solution...");
    auto goal_handle_future = trajectory_client_->async_send_goal(goal_msg);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to send trajectory");
      return false;
    }
    
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected");
      return false;
    }
    
    // Wait for result
    auto result_future = trajectory_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "\n✅ Robot moved to IK solution!");
    RCLCPP_INFO(get_logger(), "The robot should now be at the equilibrium pose from config");
    return true;
  }
  
  // Member variables
  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  std::vector<double> ik_solution_;
  std::vector<double> target_position_;
  std::vector<double> target_orientation_;
  bool current_positions_received_ = false;
  
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::Frame tool_offset_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<IKTester>();
    rclcpp::spin_some(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}