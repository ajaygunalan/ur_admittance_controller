#include "admittance_node.hpp"
#include <chrono>
#include <thread>
#include <unordered_map>

namespace ur_admittance_controller {


// Helper function to map joint states from JointState message to internal vector
void AdmittanceNode::map_joint_states(const sensor_msgs::msg::JointState& msg, bool) {
  // Tier 1: Joint configuration must be consistent
  ENSURE(params_.joints.size() == 6, "UR robot must have exactly 6 joints configured");
  
  // One-time setup of joint mapping
  static const auto joint_map = [this]() {
    std::unordered_map<std::string, size_t> map;
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      map[params_.joints[i]] = i;
    }
    return map;
  }();
  
  // Direct mapping
  for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
    if (auto it = joint_map.find(msg.name[i]); it != joint_map.end()) {
      q_current_[it->second] = msg.position[i];
    }
  }
}

void AdmittanceNode::initializeParameters() {
  param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
      this->get_node_parameters_interface());
  params_ = param_listener_->get_params();
  
  parameter_cb_handle_ = add_on_set_parameters_callback([this](const auto& params) {
    auto result = param_listener_->update(params);
    if (result.successful) {
      params_ = param_listener_->get_params();
      update_admittance_parameters();
    }
    return result;
  });
  
  update_admittance_parameters();
}

void AdmittanceNode::setupROSInterfaces() {
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/F_P_B", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg) { wrench_callback(msg); });
      
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,  // Queue size = 1 for always fresh data
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { joint_state_callback(msg); });
      
  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/admittance_node/desired_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { desired_pose_callback(msg); });
  
  velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 10);
}




void AdmittanceNode::wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  F_P_B << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
            msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  
  
  // Log wrench if non-zero
  double force_norm = F_P_B.head<3>().norm();
  double torque_norm = F_P_B.tail<3>().norm();
  if (force_norm > 0.1 || torque_norm > 0.1) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "External wrench - Force: %.2f N [%.2f, %.2f, %.2f], Torque: %.2f Nm [%.2f, %.2f, %.2f]",
      force_norm, F_P_B(0), F_P_B(1), F_P_B(2),
      torque_norm, F_P_B(3), F_P_B(4), F_P_B(5));
  }
}

void AdmittanceNode::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(), "Joint states received - robot is online");
  
  joint_states_received_ = true;
  map_joint_states(*msg);
}

// Handle desired pose updates from ROS2 topic
void AdmittanceNode::desired_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  // Only update position, keep orientation fixed at equilibrium
  X_BP_desired.translation() << msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z;
  
  // Orientation remains fixed at equilibrium value
  
  RCLCPP_DEBUG(get_logger(), "Desired position updated to [%.3f, %.3f, %.3f]",
               msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

Status AdmittanceNode::control_cycle() {
  if (!joint_states_received_) {
    RCLCPP_WARN_ONCE(get_logger(), "Waiting for initial joint states...");
    return {};  // Not an error, just waiting for data
  }
  
  // Forward kinematics - explicit error handling (Drake style)
  if (auto fk_status = get_X_BP_current(); !fk_status) {
    // Return the error directly - clear and debuggable
    return fk_status;
  }
  
  // These don't return errors (yet)
  compute_pose_error();
  compute_admittance();
  limit_to_workspace();
  
  // Inverse kinematics - explicit error handling
  if (auto ik_status = compute_and_pub_joint_velocities(); !ik_status) {
    // Return the error directly - no hidden control flow
    return ik_status;
  }
  
  return {};  // Success
}


void AdmittanceNode::initialize() {
  // Tier 1: Parameters must be loaded before initialization
  ENSURE(param_listener_ != nullptr, "Parameter listener not initialized");
  
  // Tier 2: Throw on setup failure
  if (auto status = load_kinematics(); !status) {
    throw std::runtime_error(status.error().message);
  }
  
  kinematics_initialized_ = true;
  
  RCLCPP_INFO_ONCE(get_logger(), 
    "Kinematics ready, equilibrium at: [%.3f, %.3f, %.3f]",
    X_BP_desired.translation()(0), X_BP_desired.translation()(1), X_BP_desired.translation()(2));
  
  RCLCPP_INFO_ONCE(get_logger(), "UR Admittance Controller ready at 100Hz - push the robot to move it!");
}


// Constructor: init params, state vectors, ROS I/O, and default equilibrium pose
AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  RCLCPP_INFO_ONCE(get_logger(), "Initializing UR Admittance Controller - 6-DOF Force-Compliant Motion Control");
  
  initializeParameters();
  initializeStateVectors();
  setupROSInterfaces();
  setDefaultEquilibrium();
}


}  // namespace ur_admittance_controller


// 1. Constructor →  Non-blocking
// 2. initialize → blocking operations (load kinematics, wait for joints)
// 3. spin_some() → get F/T, joint position, desired pose via callbacks
// 4. control_cycle() → compute admittance and send  joint vel to the robot
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  
  // Tier 1: Node must be successfully created
  ENSURE(node != nullptr, "Failed to create admittance node");
  
  // Tier 2: Setup failures throw exceptions
  try {
    node->initialize();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), 
                 "Failed to initialize admittance controller: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  rclcpp::Rate loop_rate(100);  // 100Hz
  
  RCLCPP_INFO_ONCE(node->get_logger(), "Starting synchronized admittance control loop at 100Hz...");
  
  while (rclcpp::ok()) {
    executor.spin_some();     
    
    auto status = node->control_cycle();
    if (!status) {
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                           "Control cycle error: %s", status.error().message.c_str());
      // Continue running for now - real-time loops should be resilient
    }
    
    loop_rate.sleep();
  }
  
  RCLCPP_INFO(node->get_logger(), "Shutting down admittance controller...");
  rclcpp::shutdown();
  return 0;
}
