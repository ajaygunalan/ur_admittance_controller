#include "admittance_node.hpp"
#include <chrono>
#include <thread>
#include <rclcpp/wait_for_message.hpp>

namespace ur_admittance_controller {


// Helper function to map joint states from JointState message to internal vector
void AdmittanceNode::map_joint_states(const sensor_msgs::msg::JointState& msg, bool warn_missing) {
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    auto it = std::find(msg.name.begin(), msg.name.end(), params_.joints[i]);
    if (it != msg.name.end() && 
        static_cast<size_t>(std::distance(msg.name.begin(), it)) < msg.position.size()) {
      q_current_[i] = msg.position[std::distance(msg.name.begin(), it)];
    } else if (warn_missing) {
      RCLCPP_WARN(get_logger(), "Joint %s not found in joint_states message!", params_.joints[i].c_str());
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
      "/wrench_tcp_base", rclcpp::SensorDataQoS(),
      std::bind(&AdmittanceNode::wrench_callback, this, std::placeholders::_1));
      
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&AdmittanceNode::joint_state_callback, this, std::placeholders::_1));
      
  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/admittance_node/desired_pose", 10,
      std::bind(&AdmittanceNode::desired_pose_callback, this, std::placeholders::_1));
  
  velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 10);
}



// Check if joint states have been received and populate q_current_
bool AdmittanceNode::checkJointStates() {
  sensor_msgs::msg::JointState msg;
  if (!rclcpp::wait_for_message(msg, shared_from_this(), "/joint_states", std::chrono::seconds(10))) {
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "Processing joint states - received %zu joints", msg.name.size());
  
  map_joint_states(msg, true);  // true = warn about missing joints
  joint_states_updated_ = true;
  
  RCLCPP_INFO(get_logger(), "Initial joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    q_current_[0], q_current_[1], q_current_[2], q_current_[3], q_current_[4], q_current_[5]);
  
  return true;
}


void AdmittanceNode::wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  Wrench_tcp_base_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  
  
  // Log wrench if non-zero
  double force_norm = Wrench_tcp_base_.head<3>().norm();
  double torque_norm = Wrench_tcp_base_.tail<3>().norm();
  if (force_norm > 0.1 || torque_norm > 0.1) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "External wrench - Force: %.2f N [%.2f, %.2f, %.2f], Torque: %.2f Nm [%.2f, %.2f, %.2f]",
      force_norm, Wrench_tcp_base_(0), Wrench_tcp_base_(1), Wrench_tcp_base_(2),
      torque_norm, Wrench_tcp_base_(3), Wrench_tcp_base_(4), Wrench_tcp_base_(5));
  }
}

void AdmittanceNode::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(), "Joint states received - robot is online");
  
  
  map_joint_states(*msg);
  joint_states_updated_ = true;
}

// Handle desired pose updates from ROS2 topic
void AdmittanceNode::desired_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  // Only update position, keep orientation fixed at equilibrium
  X_tcp_base_desired_.translation() << msg->pose.position.x,
                                       msg->pose.position.y,
                                       msg->pose.position.z;
  
  // Orientation remains fixed at equilibrium value
  
  RCLCPP_DEBUG(get_logger(), "Desired position updated to [%.3f, %.3f, %.3f]",
               msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

// Constructor: init params, state vectors, ROS I/O, and default equilibrium pose
AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  RCLCPP_INFO(get_logger(), "Initializing UR Admittance Controller - 6-DOF Force-Compliant Motion Control");
  
  initializeParameters();
  initializeStateVectors();
  setupROSInterfaces();
  setDefaultEquilibrium();
}

void AdmittanceNode::initialize() {
  if (!load_kinematics()) {
    RCLCPP_ERROR(get_logger(), "Kinematics loading failed");
  }
  
  if (!checkJointStates()) {
    RCLCPP_ERROR(get_logger(), "Joint states not received - is robot driver running?");
  }
  
  RCLCPP_INFO(get_logger(), 
    "Kinematics ready, equilibrium at: [%.3f, %.3f, %.3f]",
    X_tcp_base_desired_.translation()(0), X_tcp_base_desired_.translation()(1), X_tcp_base_desired_.translation()(2));
  
  RCLCPP_INFO(get_logger(), "UR Admittance Controller ready at 100Hz - push the robot to move it!");
}


void AdmittanceNode::control_cycle() {
  get_X_tcp_base_current_();
  compute_admittance();
  limit_to_workspace();
  send_commands_to_robot();
}

// Send velocity commands to robot
void AdmittanceNode::send_commands_to_robot() {
  static std::vector<double> last_valid_velocities(params_.joints.size(), 0.0);
  
  // Fallback to graceful deceleration on IK failure
  if (!compute_joint_velocities(V_tcp_base_commanded_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "IK velocity solver failed, using gradual deceleration");
    constexpr double decay_factor = 0.8;  // 20% speed reduction per cycle
    std::transform(last_valid_velocities.begin(), last_valid_velocities.end(), 
                   q_dot_cmd_.begin(), 
                   [decay_factor](double v) { return v * decay_factor; });
  }
  
  // Cache for next failure recovery
  last_valid_velocities = q_dot_cmd_;
  
  // Log joint velocities
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
    "Joint velocities: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s",
    q_dot_cmd_[0], q_dot_cmd_[1], q_dot_cmd_[2], 
    q_dot_cmd_[3], q_dot_cmd_[4], q_dot_cmd_[5]);
  
  // Publish to velocity controller
  velocity_msg_.data = q_dot_cmd_;
  velocity_pub_->publish(velocity_msg_);
}

}  // namespace ur_admittance_controller


// 1. Constructor →  Non-blocking
// 2. initialize → blocking operations (load kinematics, wait for joints)
// 3. spin_some() → get F/T, joint position, desired pose via callbacks
// 4. control_cycle() → compute admittance and send  joint vel to the robot
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  node->initialize();
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  rclcpp::Rate loop_rate(100);  // 100Hz
  
  RCLCPP_INFO(node->get_logger(), "Starting synchronized admittance control loop at 100Hz...");
  
  while (rclcpp::ok()) {
    executor.spin_some();     
    node->control_cycle();    
    loop_rate.sleep();
  }
  
  RCLCPP_INFO(node->get_logger(), "Shutting down admittance controller...");
  rclcpp::shutdown();
  return 0;
}
