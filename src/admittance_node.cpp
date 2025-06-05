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
  
  parameter_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& parameters) {
      auto result = param_listener_->update(parameters);
      if (result.successful) {
        params_ = param_listener_->get_params();
        this->update_admittance_parameters();
        RCLCPP_DEBUG(get_logger(), "Parameters updated - admittance matrices reconfigured");
      }
      return result;
    });
  
  this->declare_parameter("robot_description", "");
  
  update_admittance_parameters();
}

void AdmittanceNode::initializeStateVectors() {
  const auto joint_count = params_.joints.size();
  
  // Joint space vectors
  q_current_.resize(joint_count, 0.0);
  q_dot_cmd_.resize(joint_count, 0.0);
  
  // Cartesian space vectors
  Wrench_tcp_base_ = Vector6d::Zero();
  V_tcp_base_commanded_ = Vector6d::Zero();
  
  // Poses
  X_tcp_base_current_ = Eigen::Isometry3d::Identity();
  X_tcp_base_desired_ = Eigen::Isometry3d::Identity();
  
  // ROS message
  velocity_msg_.data.resize(joint_count);
  
  // Control limits
  workspace_limits_ << -0.5, 0.5, -0.5, 0.5, 0.0, 0.7;
  arm_max_vel_ = 1.5;
  arm_max_acc_ = 1.0;
  admittance_ratio_ = 1.0;
}

void AdmittanceNode::setupROSInterfaces() {
  // Subscribers
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench_tcp_base", rclcpp::SensorDataQoS(),
      std::bind(&AdmittanceNode::wrench_callback, this, std::placeholders::_1));
      
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&AdmittanceNode::joint_state_callback, this, std::placeholders::_1));
      
  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/admittance_node/desired_pose", 10,
      std::bind(&AdmittanceNode::desired_pose_callback, this, std::placeholders::_1));
  
  // Publishers
  velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 1);
}

void AdmittanceNode::setDefaultEquilibrium() {
  declare_parameter("equilibrium.position", std::vector<double>{0.1, 0.4, 0.5});
  declare_parameter("equilibrium.orientation", std::vector<double>{0.0, 0.0, 0.0, 1.0});
  
  auto eq_pos = get_parameter("equilibrium.position").as_double_array();
  auto eq_ori = get_parameter("equilibrium.orientation").as_double_array();
  
  X_tcp_base_desired_.translation() << eq_pos[0], eq_pos[1], eq_pos[2];
  X_tcp_base_desired_.linear() = Eigen::Quaterniond(eq_ori[3], eq_ori[0], eq_ori[1], eq_ori[2]).toRotationMatrix();
  
  RCLCPP_DEBUG(get_logger(), "Equilibrium pose set: position=[%.3f, %.3f, %.3f]", 
               eq_pos[0], eq_pos[1], eq_pos[2]);
}

void AdmittanceNode::initialize() {
  if (!load_kinematics()) RCLCPP_ERROR(get_logger(), "Kinematics loading failed");
  
  if (!checkJointStates()) RCLCPP_ERROR(get_logger(), "Joint states not received - is robot driver running?");
  
  // Create control timer - this replaces the manual while loop
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),  // 100Hz
    std::bind(&AdmittanceNode::control_cycle, this)
  );
  
  RCLCPP_INFO(get_logger(), "UR Admittance Controller ready - push the robot to move it!");
  RCLCPP_INFO(get_logger(), "Control timer started at 100Hz");
}

void AdmittanceNode::control_cycle() {
  // Monitor timing performance (optional debug)
  static auto last_time = get_clock()->now();
  auto current_time = get_clock()->now();
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                        "Control cycle period: %.3fms",
                        (current_time - last_time).seconds() * 1000);
  last_time = current_time;
  
  computeForwardKinematics();
  compute_admittance();
  limit_to_workspace();
  send_commands_to_robot();
}

void AdmittanceNode::wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  Wrench_tcp_base_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
}

void AdmittanceNode::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  if (!joint_states_received_) {
    joint_states_received_ = true;
    RCLCPP_INFO_ONCE(get_logger(), "Joint states received - robot is online");
  }

  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), params_.joints[i]);
    if (it != msg->name.end()) {
      const size_t idx = std::distance(msg->name.begin(), it);
      if (idx < msg->position.size()) {
        q_current_[i] = msg->position[idx];
      }
    }
  }
  
  joint_states_updated_ = true;
}

// Initialize KDL kinematics from URDF
bool AdmittanceNode::load_kinematics() {
  std::string urdf_string;
  if (!get_parameter("robot_description", urdf_string) || urdf_string.empty()) {
    RCLCPP_ERROR(get_logger(), "robot_description parameter not found or empty");
    return false;
  }

  if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
    RCLCPP_ERROR(get_logger(), "Failed to parse URDF into KDL tree");
    return false;
  }

  if (!kdl_tree_.getChain(params_.base_link, params_.tip_link, kdl_chain_)) {
    RCLCPP_ERROR(get_logger(), "Failed to extract kinematic chain: %s -> %s",
                 params_.base_link.c_str(), params_.tip_link.c_str());
    return false;
  }

  // Initialize solvers
  constexpr double PRECISION = 1e-5;
  constexpr int MAX_ITERATIONS = 150;
  constexpr double DAMPING = 0.01;
  
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_, PRECISION, MAX_ITERATIONS);
  ik_vel_solver_->setLambda(DAMPING);
  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  // Pre-allocate arrays
  num_joints_ = kdl_chain_.getNrOfJoints();
  q_kdl_.resize(num_joints_);
  v_kdl_.resize(num_joints_);

  RCLCPP_DEBUG(get_logger(), "KDL kinematics ready: %s -> %s (%zu joints)",
               params_.base_link.c_str(), params_.tip_link.c_str(), num_joints_);
  return true;
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
  
  // Publish to velocity controller
  velocity_msg_.data = q_dot_cmd_;
  velocity_pub_->publish(velocity_msg_);
}

// Apply workspace limits to Cartesian motion
void AdmittanceNode::limit_to_workspace() {
  static constexpr double BUFFER = 0.01;  // Safety margin before warnings
  const auto& pos = X_tcp_base_current_.translation();
  
  // Enforce per-axis workspace boundaries
  for (size_t i = 0; i < 3; ++i) {
    const double min = workspace_limits_[i * 2];
    const double max = workspace_limits_[i * 2 + 1];
    const bool at_min = pos[i] <= min;
    const bool at_max = pos[i] >= max;
    
    // Warn if outside buffer zone
    if ((at_min && pos[i] < min - BUFFER) || (at_max && pos[i] > max + BUFFER)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Out of workspace: %c = %.3f not in [%.3f, %.3f]",
                           "xyz"[i], pos[i], min, max);
    }
    
    // Prevent motion beyond limits
    if (at_min) V_tcp_base_commanded_[i] = std::max(0.0, V_tcp_base_commanded_[i]);
    if (at_max) V_tcp_base_commanded_[i] = std::min(0.0, V_tcp_base_commanded_[i]);
  }
  
  // Cap velocity magnitude
  if (auto n = V_tcp_base_commanded_.head<3>().norm(); n > arm_max_vel_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "High velocity: %.3f m/s", n);
    V_tcp_base_commanded_.head<3>() *= (arm_max_vel_ / n);
  }
}

// Check if joint states have been received
bool AdmittanceNode::checkJointStates() {
  sensor_msgs::msg::JointState msg;
  return rclcpp::wait_for_message(msg, shared_from_this(), "/joint_states", std::chrono::seconds(10));
}




}  // namespace ur_admittance_controller



  // How It Works Now:

  // 1. main() creates node and calls initialize()
  // 2. initialize() creates a 100Hz timer
  // 3. rclcpp::spin(node) handles all callbacks:
  //   - Timer callback (control_cycle() every 10ms)
  //   - Subscription callbacks (wrench, joint_states)
  //   - Parameter callbacks
  // 4. No manual while loop or rate.sleep() needed!

  // The control loop now runs automatically at 100Hz through the ROS2 timer system! 
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  node->initialize();
  
  RCLCPP_INFO(node->get_logger(), "Admittance controller running...");
  rclcpp::spin(node);  // Timer handles everything!
  
  rclcpp::shutdown();
  return 0;
}