/**
 * @file verify_poses.cpp
 * @brief Verify FK/IK for home and equilibrium poses using KDL
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
#include <cmath>

class PoseVerifier : public rclcpp::Node {
public:
  PoseVerifier() : Node("pose_verifier"),
    joint_names_({"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}),
    home_joints_(6) {
    
    if (!setupRobotModel()) {
      throw std::runtime_error("Failed to setup robot model");
    }
    
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&PoseVerifier::jointCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Waiting for robot home position from /joint_states...");
    
    // Wait for joint states
    while (!home_joints_received_ && rclcpp::ok()) {
      rclcpp::spin_some(get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    verifyPoses();
  }

private:
  bool setupRobotModel() {
    auto param_client = create_client<rcl_interfaces::srv::GetParameters>(
      "/robot_state_publisher/get_parameters");
    
    if (!param_client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "robot_state_publisher parameter service not available");
      return false;
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_description"};
    
    auto future = param_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, 
        std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to get robot_description");
      return false;
    }
    
    auto response = future.get();
    if (response->values.empty() || 
        response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      RCLCPP_ERROR(get_logger(), "Invalid robot_description parameter");
      return false;
    }
    
    // Parse URDF and extract chains
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(response->values[0].string_value, tree) ||
        !tree.getChain("base_link", "wrist_3_link", kdl_chain_)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF or extract chain");
      return false;
    }
    
    // Get fixed transform to tool_payload
    KDL::Chain tool_chain;
    if (!tree.getChain("wrist_3_link", "tool_payload", tool_chain)) {
      RCLCPP_ERROR(get_logger(), "Failed to extract tool_payload chain");
      return false;
    }
    
    KDL::ChainFkSolverPos_recursive tool_fk(tool_chain);
    KDL::JntArray zero_joint(tool_chain.getNrOfJoints());
    tool_fk.JntToCart(zero_joint, ft_offset_);
    
    RCLCPP_INFO(get_logger(), "Tool offset from wrist_3_link: [%.3f, %.3f, %.3f]", 
                ft_offset_.p.x(), ft_offset_.p.y(), ft_offset_.p.z());
    
    // Create solvers
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
    
    RCLCPP_INFO(get_logger(), "Robot model loaded successfully with %d joints", 
                kdl_chain_.getNrOfJoints());
    return true;
  }
  
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (home_joints_received_) return;
    
    // Extract joint positions efficiently
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) return;
      
      size_t idx = std::distance(msg->name.begin(), it);
      if (idx >= msg->position.size()) return;
      
      home_joints_[i] = msg->position[idx];
    }
    
    home_joints_received_ = true;
    RCLCPP_INFO(get_logger(), "Got home position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                home_joints_[0], home_joints_[1], home_joints_[2],
                home_joints_[3], home_joints_[4], home_joints_[5]);
  }
  
  void printJointAngles(const std::string& title, const KDL::JntArray& joints) {
    std::cout << "\n" << title << std::endl;
    for (size_t i = 0; i < 6; ++i) {
      std::cout << std::setw(22) << joint_names_[i] << ": " 
                << std::setw(8) << std::fixed << std::setprecision(4) << joints(i) 
                << " rad (" << std::setw(8) << std::setprecision(2) 
                << joints(i) * 180.0 / M_PI << "°)" << std::endl;
    }
  }
  
  void printFrame(const std::string& prefix, const KDL::Frame& frame) {
    std::cout << "\n   " << prefix << " position: [" 
              << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << "]" << std::endl;
    
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    std::cout << "   " << prefix << " orientation (wxyz): [" 
              << w << ", " << x << ", " << y << ", " << z << "]" << std::endl;
  }
  
  KDL::Frame computeToolFrame(const KDL::JntArray& joints) {
    KDL::Frame wrist3_frame;
    fk_solver_->JntToCart(joints, wrist3_frame);
    return wrist3_frame * ft_offset_;
  }
  
  void verifyPoses() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "POSE VERIFICATION FOR UR5e WITH F/T SENSOR (KDL Model)" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    std::cout << "\nNote: tool_payload is attached to wrist_3_link via fixed joint" << std::endl;
    std::cout << "Tool offset from wrist_3: [" << ft_offset_.p.x() << ", " 
              << ft_offset_.p.y() << ", " << ft_offset_.p.z() << "] m" << std::endl;
    
    // 1. HOME POSE
    std::cout << "\n1. HOME POSE (from robot):" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
    
    KDL::JntArray q_home(6);
    for (size_t i = 0; i < 6; ++i) q_home(i) = home_joints_[i];
    
    printJointAngles("", q_home);
    printFrame("Tool", computeToolFrame(q_home));
    
    // 2. EQUILIBRIUM POSE
    std::cout << "\n2. EQUILIBRIUM POSE (IK for [0.1, 0.4, 0.5]):" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
    
    const std::vector<double> target_pos = {0.1, 0.4, 0.5};
    std::cout << "   Target position: [" << target_pos[0] << ", " 
              << target_pos[1] << ", " << target_pos[2] << "]" << std::endl;
    
    // Setup target frames
    KDL::Frame target_tool_frame(KDL::Rotation::Identity(), 
                                 KDL::Vector(target_pos[0], target_pos[1], target_pos[2]));
    KDL::Frame target_wrist3_frame = target_tool_frame * ft_offset_.Inverse();
    
    // IK with standard initial guess
    KDL::JntArray q_init(6), q_sol(6);
    q_init(0) = 0.0; q_init(1) = -1.57; q_init(2) = 1.57;
    q_init(3) = -1.57; q_init(4) = -1.57; q_init(5) = 0.0;
    
    int ik_result = ik_solver_->CartToJnt(q_init, target_wrist3_frame, q_sol);
    
    if (ik_result >= 0) {
      // Print solution
      std::cout << "\n   IK solution: [";
      for (size_t i = 0; i < 6; ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << std::fixed << std::setprecision(3) << q_sol(i);
      }
      std::cout << "]" << std::endl;
      
      // Verify with FK
      KDL::Frame fk_tool_frame = computeToolFrame(q_sol);
      std::cout << "   FK gives tool at: [" << fk_tool_frame.p.x() << ", " 
                << fk_tool_frame.p.y() << ", " << fk_tool_frame.p.z() << "]" << std::endl;
      std::cout << "   Target was: [" << target_pos[0] << ", " 
                << target_pos[1] << ", " << target_pos[2] << "]" << std::endl;
      
      double error = (fk_tool_frame.p - target_tool_frame.p).Norm();
      std::cout << "   Position error: " << std::fixed << std::setprecision(6) 
                << error << " m" << std::endl;
      
      printJointAngles("   Joint angles:", q_sol);
    } else {
      std::cout << "   IK failed to converge (error code: " << ik_result << ")" << std::endl;
    }
    
    std::cout << "\n" << std::string(80, '=') << std::endl;
  }
  
  // Member variables
  const std::vector<std::string> joint_names_;
  std::vector<double> home_joints_;
  bool home_joints_received_ = false;
  
  // KDL structures
  KDL::Chain kdl_chain_;
  KDL::Frame ft_offset_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<PoseVerifier>();
    rclcpp::spin_some(node);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}