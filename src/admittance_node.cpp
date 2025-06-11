#include "admittance_node.hpp"
#include <chrono>
#include <thread>
#include <rclcpp/wait_for_message.hpp>

namespace ur_admittance_controller {

AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  RCLCPP_INFO(get_logger(), "Initializing UR Admittance Controller - 6-DOF Force-Compliant Motion Control");
  
  initializeParameters();
  initializeStateVectors();
  setupROSInterfaces();
  setDefaultEquilibrium();
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

void AdmittanceNode::initialize() {
  if (!load_kinematics()) RCLCPP_ERROR(get_logger(), "Kinematics loading failed");
  
  if (!checkJointStates()) RCLCPP_ERROR(get_logger(), "Joint states not received - is robot driver running?");
  
  computeForwardKinematics();
  // Don't overwrite the equilibrium that was set in constructor!
  // X_tcp_base_desired_ = X_tcp_base_current_;  // REMOVED - This was causing the error growth
  
  RCLCPP_INFO(get_logger(), 
    "Initial TCP: [%.3f, %.3f, %.3f], Equilibrium: [%.3f, %.3f, %.3f]",
    X_tcp_base_current_.translation()(0), X_tcp_base_current_.translation()(1), X_tcp_base_current_.translation()(2),
    X_tcp_base_desired_.translation()(0), X_tcp_base_desired_.translation()(1), X_tcp_base_desired_.translation()(2));
  
  // Timer removed - using manual spin_some() pattern for proper synchronization
  // control_timer_ = create_wall_timer(...);  // REMOVED for synchronized control
  
  RCLCPP_INFO(get_logger(), "UR Admittance Controller ready at 100Hz - push the robot to move it!");
}

void AdmittanceNode::control_cycle() {
  // Safety check: Warn if sensor data is stale (>50ms old)
  constexpr double STALE_DATA_THRESHOLD = 0.05;  // 5 control cycles
  auto now = this->get_clock()->now();
  
  if (last_wrench_time_.nanoseconds() > 0 && 
      (now - last_wrench_time_).seconds() > STALE_DATA_THRESHOLD) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "F/T sensor data is stale (%.1fms old) - check sensor connection",
      (now - last_wrench_time_).seconds() * 1000);
  }
  
  if (last_joint_state_time_.nanoseconds() > 0 && 
      (now - last_joint_state_time_).seconds() > STALE_DATA_THRESHOLD) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Joint state data is stale (%.1fms old) - check robot driver",
      (now - last_joint_state_time_).seconds() * 1000);
  }
  
  computeForwardKinematics();
  compute_admittance();
  limit_to_workspace();
  send_commands_to_robot();
}

void AdmittanceNode::wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  Wrench_tcp_base_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  
  // Track data freshness
  last_wrench_time_ = msg->header.stamp;
  
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
  
  // Track data freshness
  last_joint_state_time_ = msg->header.stamp;
  
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
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Joint velocities: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s",
    q_dot_cmd_[0], q_dot_cmd_[1], q_dot_cmd_[2], 
    q_dot_cmd_[3], q_dot_cmd_[4], q_dot_cmd_[5]);
  
  // Publish to velocity controller
  velocity_msg_.data = q_dot_cmd_;
  velocity_pub_->publish(velocity_msg_);
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

}  // namespace ur_admittance_controller

// How It Works Now:
// 1. main() creates node and calls initialize()
// 2. Manual spin_some() loop processes all sensor callbacks
// 3. control_cycle() runs immediately after with synchronized data
// 4. This ensures temporal alignment of wrench and joint state data
// The control loop now runs at 100Hz with guaranteed sensor synchronization!
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  node->initialize();
  
  // Create single-threaded executor for deterministic callback processing
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  // Control loop rate
  rclcpp::Rate loop_rate(100);  // 100Hz
  
  RCLCPP_INFO(node->get_logger(), "Starting synchronized admittance control loop at 100Hz...");
  
  // Manual control loop with synchronized sensor processing
  while (rclcpp::ok()) {
    // Process ALL pending callbacks (wrench, joint_states, parameters)
    // This ensures all sensor data is updated before control computation
    executor.spin_some();
    
    // Run control cycle with freshest synchronized data
    node->control_cycle();
    
    // Sleep to maintain 100Hz rate
    loop_rate.sleep();
  }
  
  RCLCPP_INFO(node->get_logger(), "Shutting down admittance controller...");
  rclcpp::shutdown();
  return 0;
}
