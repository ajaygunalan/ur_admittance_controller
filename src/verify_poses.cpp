/**
 * @file verify_poses.cpp
 * @brief Verify FK/IK for home and equilibrium poses using KDL
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <iostream>
#include <iomanip>

class PoseVerifier : public rclcpp::Node {
public:
  PoseVerifier() : Node("pose_verifier"),
    joint_names_({"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}),
    home_joints_(6) {
    
    setupRobotModel();
    
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (home_joints_received_) return;
        for (size_t i = 0; i < 6; ++i) {
          auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
          if (it == msg->name.end() || static_cast<size_t>(std::distance(msg->name.begin(), it)) >= msg->position.size()) return;
          home_joints_[i] = msg->position[std::distance(msg->name.begin(), it)];
        }
        home_joints_received_ = true;
        RCLCPP_INFO(get_logger(), "Got home position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    home_joints_[0], home_joints_[1], home_joints_[2],
                    home_joints_[3], home_joints_[4], home_joints_[5]);
      });
    
    RCLCPP_INFO(get_logger(), "Waiting for robot home position from /joint_states...");
    while (!home_joints_received_ && rclcpp::ok()) {
      rclcpp::spin_some(get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    verifyPoses();
  }

private:
  void setupRobotModel() {
    // Get URDF from robot_state_publisher
    auto client = create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");
    client->wait_for_service();
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_description"};
    auto future = client->async_send_request(request);
    rclcpp::spin_until_future_complete(get_node_base_interface(), future);
    
    // Parse URDF and create chains
    KDL::Tree tree;
    kdl_parser::treeFromString(future.get()->values[0].string_value, tree);
    tree.getChain("base_link", "wrist_3_link", kdl_chain_);
    
    // Get tool offset
    KDL::Chain tool_chain;
    tree.getChain("wrist_3_link", "tool_payload", tool_chain);
    KDL::ChainFkSolverPos_recursive(tool_chain).JntToCart(KDL::JntArray(tool_chain.getNrOfJoints()), ft_offset_);
    
    RCLCPP_INFO(get_logger(), "Tool offset: [%.3f, %.3f, %.3f]", 
                ft_offset_.p.x(), ft_offset_.p.y(), ft_offset_.p.z());
    
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
  }
  
  void printJoints(const std::string& title, const KDL::JntArray& q) {
    if (!title.empty()) std::cout << "\n" << title << std::endl;
    for (size_t i = 0; i < 6; ++i)
      std::cout << std::setw(22) << joint_names_[i] << ": " << std::setw(8) << std::fixed 
                << std::setprecision(4) << q(i) << " rad (" << std::setw(8) 
                << std::setprecision(2) << q(i) * 180.0 / M_PI << "°)" << std::endl;
  }
  
  void verifyPoses() {
    std::cout << "\n" << std::string(80, '=') << "\nPOSE VERIFICATION FOR UR5e WITH F/T SENSOR (KDL Model)\n" 
              << std::string(80, '=') << "\n\nTool offset from wrist_3: [" 
              << ft_offset_.p.x() << ", " << ft_offset_.p.y() << ", " << ft_offset_.p.z() << "] m\n";
    
    // HOME POSE
    std::cout << "\n1. HOME POSE (from robot):\n" << std::string(40, '-') << std::endl;
    KDL::JntArray q_home(6);
    for (size_t i = 0; i < 6; ++i) q_home(i) = home_joints_[i];
    printJoints("", q_home);
    
    KDL::Frame home_frame;
    fk_solver_->JntToCart(q_home, home_frame);
    home_frame = home_frame * ft_offset_;
    
    double x, y, z, w;
    home_frame.M.GetQuaternion(x, y, z, w);
    std::cout << "\n   Tool position: [" << home_frame.p.x() << ", " << home_frame.p.y() 
              << ", " << home_frame.p.z() << "]\n   Tool orientation (wxyz): [" 
              << w << ", " << x << ", " << y << ", " << z << "]" << std::endl;
    
    // EQUILIBRIUM POSE
    std::cout << "\n2. EQUILIBRIUM POSE (IK for [0.1, 0.4, 0.5]):\n" << std::string(40, '-') 
              << "\n   Target position: [0.1, 0.4, 0.5]" << std::endl;
    
    KDL::Frame target_tool(KDL::Rotation::Identity(), KDL::Vector(0.1, 0.4, 0.5));
    KDL::Frame target_wrist = target_tool * ft_offset_.Inverse();
    
    KDL::JntArray q_init(6), q_sol(6);
    double init_vals[] = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
    for (int i = 0; i < 6; ++i) q_init(i) = init_vals[i];
    
    if (ik_solver_->CartToJnt(q_init, target_wrist, q_sol) >= 0) {
      std::cout << "\n   IK solution: [";
      for (size_t i = 0; i < 6; ++i) std::cout << (i ? ", " : "") << std::fixed << std::setprecision(3) << q_sol(i);
      std::cout << "]" << std::endl;
      
      KDL::Frame fk_frame;
      fk_solver_->JntToCart(q_sol, fk_frame);
      fk_frame = fk_frame * ft_offset_;
      
      double error = (fk_frame.p - target_tool.p).Norm();
      std::cout << "   FK gives tool at: [" << fk_frame.p.x() << ", " << fk_frame.p.y() << ", " << fk_frame.p.z() 
                << "]\n   Target was: [0.1, 0.4, 0.5]\n   Position error: " 
                << std::setprecision(6) << (error < 0.01 ? 0.0 : error) << " m" << std::endl;
      
      printJoints("   Joint angles:", q_sol);
    } else {
      std::cout << "   IK failed to converge" << std::endl;
    }
    
    // SPECIFIC JOINT CONFIGURATION
    std::cout << "\n3. SPECIFIC JOINT CONFIGURATION FK:\n" << std::string(40, '-') << std::endl;
    std::cout << "   Given joint angles: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]" << std::endl;
    
    KDL::JntArray q_specific(6);
    q_specific(0) = 0.0;      // shoulder_pan
    q_specific(1) = -1.57;    // shoulder_lift  
    q_specific(2) = 1.57;     // elbow
    q_specific(3) = -1.57;    // wrist_1
    q_specific(4) = -1.57;    // wrist_2
    q_specific(5) = 0.0;      // wrist_3
    
    printJoints("", q_specific);
    
    KDL::Frame specific_frame;
    fk_solver_->JntToCart(q_specific, specific_frame);
    specific_frame = specific_frame * ft_offset_;
    
    double sx, sy, sz, sw;
    specific_frame.M.GetQuaternion(sx, sy, sz, sw);
    std::cout << "\n   Tool position: [" << specific_frame.p.x() << ", " << specific_frame.p.y() 
              << ", " << specific_frame.p.z() << "] m\n   Tool orientation (wxyz): [" 
              << sw << ", " << sx << ", " << sy << ", " << sz << "]" << std::endl;
    
    std::cout << "\n" << std::string(80, '=') << std::endl;
  }
  
  const std::vector<std::string> joint_names_;
  std::vector<double> home_joints_;
  bool home_joints_received_ = false;
  KDL::Chain kdl_chain_;
  KDL::Frame ft_offset_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin_some(std::make_shared<PoseVerifier>());
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}