#include "admittance_node.hpp"
#include <chrono>
#include <thread>

namespace ur_admittance_controller {

AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  RCLCPP_INFO(get_logger(), "Initializing UR Admittance Controller - 6-DOF Force-Compliant Motion Control");
  
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
  std::string robot_description;
  this->get_parameter("robot_description", robot_description);
  
  if (!robot_description.empty()) {
    kinematics_initialized_ = load_kinematics(robot_description);
    if (!kinematics_initialized_) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize kinematics from robot_description parameter");
    }
  } else {
    RCLCPP_WARN(get_logger(), "robot_description parameter is empty - kinematics not initialized");
  }
  
  const auto joint_count = params_.joints.size();
  q_current_.resize(joint_count, 0.0);
  q_dot_cmd_.resize(joint_count, 0.0);
  
  Wrench_tcp_base_ = Vector6d::Zero();
  V_tcp_base_commanded_ = Vector6d::Zero();
  V_tcp_base_desired_ = Vector6d::Zero();
  
  X_tcp_base_current_ = Eigen::Isometry3d::Identity();
  X_tcp_base_desired_ = Eigen::Isometry3d::Identity();
  
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
      "/forward_velocity_controller/commands", 1);
  
  update_admittance_parameters();

  velocity_msg_.data.resize(params_.joints.size());
  
  workspace_limits_ << -1.0, 1.0, -1.0, 1.0, 0.0, 1.5;
  arm_max_vel_ = 0.5;
  arm_max_acc_ = 1.0;
  admittance_ratio_ = 1.0;
  
  declare_parameter("equilibrium.position", std::vector<double>{0.1, 0.4, 0.5});
  declare_parameter("equilibrium.orientation", std::vector<double>{0.0, 0.0, 0.0, 1.0});
  
  auto eq_pos = get_parameter("equilibrium.position").as_double_array();
  auto eq_ori = get_parameter("equilibrium.orientation").as_double_array();
  
  X_tcp_base_desired_.translation() << eq_pos[0], eq_pos[1], eq_pos[2];
  X_tcp_base_desired_.linear() = Eigen::Quaterniond(eq_ori[3], eq_ori[0], eq_ori[1], eq_ori[2]).toRotationMatrix();
  
  RCLCPP_DEBUG(get_logger(), "Equilibrium pose set: position=[%.3f, %.3f, %.3f]", 
               eq_pos[0], eq_pos[1], eq_pos[2]);
}

bool AdmittanceNode::initialize() {
  if (!kinematics_initialized_) {
    RCLCPP_ERROR(get_logger(), 
                 "Kinematics not initialized. Check that:\n"
                 "  1. robot_state_publisher is running\n"
                 "  2. robot_description parameter contains valid URDF\n"
                 "  3. base_link (%s) and tip_link (%s) exist in URDF",
                 params_.base_link.c_str(), params_.tip_link.c_str());
    return false;
  }
  
  try {
    wait_for_robot_ready();
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(get_logger(), "Initialization failed: %s", e.what());
    return false;
  }
  
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "Shutdown requested during initialization");
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "=== Initialization Complete - Ready for Control ===");
  RCLCPP_INFO(get_logger(), "UR Admittance Controller ready - push the robot to move it!");
  return true;
}

void AdmittanceNode::control_cycle() {
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
bool AdmittanceNode::load_kinematics(const std::string& urdf_string) {
  if (urdf_string.empty()) {
    RCLCPP_WARN(get_logger(), "Empty URDF string provided");
    return false;
  }

  try {
    // Parse URDF string into KDL kinematic tree structure
    if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF into KDL tree");
      return false;
    }

    // Extract kinematic chain from base to end-effector
    if (!kdl_tree_.getChain(params_.base_link, params_.tip_link, kdl_chain_)) {
      RCLCPP_ERROR(get_logger(), "Failed to extract kinematic chain: %s -> %s",
                   params_.base_link.c_str(), params_.tip_link.c_str());
      return false;
    }

    // Configure WDLS (Weighted Damped Least Squares) inverse velocity solver
    constexpr double kPrecisionThreshold = 1e-5;  // Convergence precision
    constexpr int kMaxIterations = 150;           // Maximum solver iterations
    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_, 
                                                                  kPrecisionThreshold, 
                                                                  kMaxIterations);

    // Set damping factor for singularity robustness
    constexpr double kDampingFactor = 0.01;
    ik_vel_solver_->setLambda(kDampingFactor);

    // Create forward kinematics solver (ROS1-style direct computation)
    fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

    // Cache number of joints and pre-allocate KDL arrays to avoid repeated allocations
    num_joints_ = kdl_chain_.getNrOfJoints();
    q_kdl_.resize(num_joints_);
    v_kdl_.resize(num_joints_);

    RCLCPP_DEBUG(get_logger(), "KDL kinematics ready: %s -> %s (%d joints, %d segments)",
                 params_.base_link.c_str(), params_.tip_link.c_str(),
                 kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfSegments());

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Kinematics initialization failed: %s", e.what());
    return false;
  }
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
  // Store last valid joint velocities for graceful degradation
  static std::vector<double> last_valid_velocities(params_.joints.size(), 0.0);
  
  // Convert Cartesian velocity to joint velocities
  if (!compute_joint_velocities(V_tcp_base_commanded_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "IK velocity solver failed, using gradual deceleration");
    
    // Gradually reduce velocity instead of sudden stop to avoid jerky motion
    constexpr double decay_factor = 0.8;  // 20% reduction per cycle
    for (size_t i = 0; i < q_dot_cmd_.size(); ++i) {
      q_dot_cmd_[i] = last_valid_velocities[i] * decay_factor;
    }
    
    // Update last valid velocities with decayed values
    last_valid_velocities = q_dot_cmd_;
  } else {
    // IK succeeded - update last valid velocities
    for (size_t i = 0; i < q_dot_cmd_.size(); ++i) {
      last_valid_velocities[i] = q_dot_cmd_[i];
    }
  }
  
  // Publish joint velocity commands (whether from IK or graceful degradation)
  for (size_t i = 0; i < velocity_msg_.data.size(); ++i) {
    velocity_msg_.data[i] = q_dot_cmd_[i];
  }
  velocity_pub_->publish(velocity_msg_);
}

// Apply workspace limits to Cartesian motion
void AdmittanceNode::limit_to_workspace() {
  // Constants for cleaner code
  static constexpr double BUFFER_ZONE = 0.01;  // 1cm buffer before warning
  static constexpr std::array<char, 3> AXIS_NAMES = {'x', 'y', 'z'};
  
  // Get current TCP position from forward kinematics
  const auto& current_position = X_tcp_base_current_.translation();
  
  // Start with desired velocity from admittance control
  V_tcp_base_commanded_ = V_tcp_base_desired_;
  // Note: Add any additional desired velocity here if needed (e.g., from trajectory planning)
  
  // Check workspace boundaries and limit velocities for each axis
  for (size_t i = 0; i < 3; ++i) {
    const double pos = current_position[i];
    const double min_limit = workspace_limits_[i * 2];
    const double max_limit = workspace_limits_[i * 2 + 1];
    
    // Check minimum boundary
    if (pos <= min_limit) {
      // Warn if significantly outside boundary
      if (pos < min_limit - BUFFER_ZONE) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Out of permitted workspace. %c = %.3f not in [%.3f, %.3f]",
                             AXIS_NAMES[i], pos, min_limit, max_limit);
      }
      // Prevent motion further outside boundary
      V_tcp_base_commanded_[i] = std::max(0.0, V_tcp_base_commanded_[i]);
    }
    
    // Check maximum boundary
    else if (pos >= max_limit) {
      // Warn if significantly outside boundary
      if (pos > max_limit + BUFFER_ZONE) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Out of permitted workspace. %c = %.3f not in [%.3f, %.3f]",
                             AXIS_NAMES[i], pos, min_limit, max_limit);
      }
      // Prevent motion further outside boundary
      V_tcp_base_commanded_[i] = std::min(0.0, V_tcp_base_commanded_[i]);
    }
  }
  
  // Limit velocity magnitude to maximum allowed
  const double velocity_norm = V_tcp_base_commanded_.head<3>().norm();
  
  if (velocity_norm > arm_max_vel_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Admittance generates fast arm movements! velocity norm: %.3f",
                         velocity_norm);
    
    // Scale velocity to stay within limits while preserving direction
    const double scaling_factor = arm_max_vel_ / velocity_norm;
    V_tcp_base_commanded_.head<3>() *= scaling_factor;
  }
}

// Wait for robot to be ready (joint states available) with timeout
void AdmittanceNode::wait_for_robot_ready() {
  // Waiting for robot to be ready - progress will be shown below
  
  // Modern ROS2 pattern: Use timeout instead of infinite loop
  const auto timeout_duration = std::chrono::seconds(10);
  const auto start_time = std::chrono::steady_clock::now();
  
  while (!joint_states_received_ && rclcpp::ok()) {
    // Check for timeout
    const auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed > timeout_duration) {
      RCLCPP_ERROR(get_logger(), 
                   "Timeout waiting for joint states after %ld seconds. "
                   "Is the robot driver running?",
                   std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
      throw std::runtime_error("Joint states timeout");
    }
    
    // Log progress less frequently (every 2 seconds instead of 1)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Waiting for joint states... (%.1f/%.1f seconds)",
                         std::chrono::duration<double>(elapsed).count(),
                         std::chrono::duration<double>(timeout_duration).count());
    
    // Process callbacks
    rclcpp::spin_some(shared_from_this());
    
    // Use shorter sleep for more responsive initialization
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
  if (joint_states_received_) {
    RCLCPP_INFO(get_logger(), "✓ Joint states received - robot is ready");
  }
}




}  // namespace ur_admittance_controller

// Main entry point - initialize ROS2 and start admittance control node
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  
  // Initialize the controller and wait for robot to be ready
  if (!node->initialize()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize admittance controller");
    rclcpp::shutdown();
    return 1;
  }
  
  // Control frequency and period (elegant like ROS1)
  const double frequency = 100.0;  // Hz
  rclcpp::Rate rate(frequency);
  node->control_period_ = rclcpp::Duration::from_seconds(1.0 / frequency);
  
  RCLCPP_INFO(node->get_logger(), "Starting control loop at %.0fHz (dt=%.3fs)...", 
              frequency, node->control_period_.seconds());
  
  // Main control loop - standard ROS2 pattern
  while (rclcpp::ok()) {
    // Process pending callbacks (subscriptions, services)
    rclcpp::spin_some(node);
    
    // Run control computation
    node->control_cycle();
    
    // Maintain 100Hz rate
    rate.sleep();
  }
  
  RCLCPP_INFO(node->get_logger(), "Shutting down UR Admittance Controller");
  rclcpp::shutdown();
  return 0;
}