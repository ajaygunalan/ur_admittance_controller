/**
 * @file verify_poses.cpp
 * @brief Verify FK/IK for home and equilibrium poses using KDL
 * 
 * C++ implementation of verify_poses.py using KDL for kinematics
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

// KDL headers
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>

class PoseVerifier : public rclcpp::Node {
public:
  PoseVerifier() : Node("pose_verifier") {
    // Initialize joint names
    joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    
    // Setup robot model
    if (!setupRobotModel()) {
      throw std::runtime_error("Failed to setup robot model");
    }
    
    // Subscribe to joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&PoseVerifier::jointCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Waiting for robot home position from /joint_states...");
    
    // Wait for initial joint states
    while (!home_joints_received_) {
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Perform verification
    verifyPoses();
  }

private:
  bool setupRobotModel() {
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
    
    // Extract chain from base_link to tool_payload (or wrist_3_link if tool not present)
    has_tool_payload_ = kdl_tree_.getChain("base_link", "tool_payload", kdl_chain_);
    if (!has_tool_payload_) {
      // Fallback to wrist_3_link if tool_payload doesn't exist
      if (!kdl_tree_.getChain("base_link", "wrist_3_link", kdl_chain_)) {
        RCLCPP_ERROR(get_logger(), "Failed to extract kinematic chain");
        return false;
      }
      RCLCPP_INFO(get_logger(), "Using wrist_3_link as end effector (tool_payload not found)");
    } else {
      RCLCPP_INFO(get_logger(), "Using tool_payload as end effector");
      
      // If we have tool_payload, also get the wrist_3 chain to compute offset
      KDL::Chain wrist_chain;
      if (kdl_tree_.getChain("base_link", "wrist_3_link", wrist_chain)) {
        // Compute transform from wrist_3 to tool
        KDL::ChainFkSolverPos_recursive wrist_fk(wrist_chain);
        KDL::ChainFkSolverPos_recursive tool_fk(kdl_chain_);
        
        KDL::JntArray zero_joints(6);
        for (int i = 0; i < 6; ++i) zero_joints(i) = 0.0;
        
        KDL::Frame wrist_frame, tool_frame;
        wrist_fk.JntToCart(zero_joints, wrist_frame);
        tool_fk.JntToCart(zero_joints, tool_frame);
        
        // ft_offset = wrist_frame.Inverse() * tool_frame
        ft_offset_ = wrist_frame.Inverse() * tool_frame;
      }
    }
    
    // Create FK and IK solvers
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
    
    RCLCPP_INFO(get_logger(), "Robot model loaded successfully with %d joints", kdl_chain_.getNrOfJoints());
    return true;
  }
  
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (home_joints_received_) return;
    
    // Extract joint positions in order
    home_joints_.resize(joint_names_.size());
    bool all_found = true;
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it != msg->name.end()) {
        size_t idx = std::distance(msg->name.begin(), it);
        if (idx < msg->position.size()) {
          home_joints_[i] = msg->position[idx];
        } else {
          all_found = false;
          break;
        }
      } else {
        all_found = false;
        break;
      }
    }
    
    if (all_found) {
      home_joints_received_ = true;
      RCLCPP_INFO(get_logger(), "Got home position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  home_joints_[0], home_joints_[1], home_joints_[2],
                  home_joints_[3], home_joints_[4], home_joints_[5]);
    }
  }
  
  void verifyPoses() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "POSE VERIFICATION FOR UR5e WITH F/T SENSOR (KDL Model)" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    if (has_tool_payload_) {
      std::cout << "\nNote: tool_payload is attached to wrist_3_link" << std::endl;
      std::cout << "Offset from wrist_3 to tool: " << std::endl;
      std::cout << "  Position: [" << ft_offset_.p.x() << ", " << ft_offset_.p.y() << ", " << ft_offset_.p.z() << "]" << std::endl;
    } else {
      std::cout << "\nNote: Using wrist_3_link as end effector (no tool_payload)" << std::endl;
    }
    
    // 1. HOME POSE
    std::cout << "\n1. HOME POSE (from robot):" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      std::cout << std::setw(22) << joint_names_[i] << ": " 
                << std::setw(8) << std::fixed << std::setprecision(4) << home_joints_[i] 
                << " rad (" << std::setw(8) << std::setprecision(2) << home_joints_[i] * 180.0 / M_PI << "°)" 
                << std::endl;
    }
    
    // Compute FK for home position
    KDL::JntArray q_home(kdl_chain_.getNrOfJoints());
    for (size_t i = 0; i < home_joints_.size(); ++i) {
      q_home(i) = home_joints_[i];
    }
    
    KDL::Frame home_frame;
    fk_solver_->JntToCart(q_home, home_frame);
    
    std::cout << "\n   Tool position: [" << home_frame.p.x() << ", " 
              << home_frame.p.y() << ", " << home_frame.p.z() << "]" << std::endl;
    
    double x, y, z, w;
    home_frame.M.GetQuaternion(x, y, z, w);
    std::cout << "   Tool orientation (wxyz): [" << w << ", " << x << ", " << y << ", " << z << "]" << std::endl;
    
    // 2. EQUILIBRIUM POSE
    std::cout << "\n2. EQUILIBRIUM POSE (IK for [0.1, 0.4, 0.5]):" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
    
    std::vector<double> target_pos = {0.1, 0.4, 0.5};
    std::cout << "   Target position: [" << target_pos[0] << ", " << target_pos[1] << ", " << target_pos[2] << "]" << std::endl;
    
    // Create target frame
    KDL::Frame target_frame;
    target_frame.p = KDL::Vector(target_pos[0], target_pos[1], target_pos[2]);
    target_frame.M = KDL::Rotation::Identity();  // Default orientation
    
    // Initial joint positions for IK
    KDL::JntArray q_init(kdl_chain_.getNrOfJoints());
    q_init(0) = 0.0;
    q_init(1) = -1.57;
    q_init(2) = 1.57;
    q_init(3) = -1.57;
    q_init(4) = -1.57;
    q_init(5) = 0.0;
    
    // Solve IK
    KDL::JntArray q_sol(kdl_chain_.getNrOfJoints());
    int ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_sol);
    
    if (ik_result >= 0) {
      std::cout << "\n   IK solution: [";
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << std::fixed << std::setprecision(3) << q_sol(i);
      }
      std::cout << "]" << std::endl;
      
      // Verify with FK
      KDL::Frame fk_frame;
      fk_solver_->JntToCart(q_sol, fk_frame);
      
      std::cout << "   FK gives tool at: [" << fk_frame.p.x() << ", " 
                << fk_frame.p.y() << ", " << fk_frame.p.z() << "]" << std::endl;
      std::cout << "   Target was: [" << target_pos[0] << ", " << target_pos[1] << ", " << target_pos[2] << "]" << std::endl;
      
      double error = (fk_frame.p - target_frame.p).Norm();
      std::cout << "   Position error: " << std::fixed << std::setprecision(6) << error << " m" << std::endl;
      
      std::cout << "\n   Joint angles:" << std::endl;
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        std::cout << std::setw(22) << joint_names_[i] << ": " 
                  << std::setw(8) << std::fixed << std::setprecision(4) << q_sol(i) 
                  << " rad (" << std::setw(8) << std::setprecision(2) << q_sol(i) * 180.0 / M_PI << "°)" 
                  << std::endl;
      }
    } else {
      std::cout << "   IK failed to converge (error code: " << ik_result << ")" << std::endl;
    }
    
    std::cout << "\n" << std::string(80, '=') << std::endl;
  }
  
  // Member variables
  std::vector<std::string> joint_names_;
  std::vector<double> home_joints_;
  bool home_joints_received_ = false;
  bool has_tool_payload_ = false;
  
  // KDL structures
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::Frame ft_offset_;  // Transform from wrist_3 to tool_payload
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<PoseVerifier>();
    // Just need to run once to verify poses
    rclcpp::spin_some(node);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}