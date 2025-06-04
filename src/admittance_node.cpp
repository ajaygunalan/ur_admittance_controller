#include "admittance_node.hpp"
#include <chrono>
#include <thread>

namespace ur_admittance_controller {

// Main constructor - initializes admittance control system
AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  RCLCPP_INFO(get_logger(), "Initializing UR Admittance Controller - 6-DOF Force-Compliant Motion Control");
  
  // Set up dynamic parameter system with auto-generated parameter library
  param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
      this->get_node_parameters_interface());
  params_ = param_listener_->get_params();
  
  // Set up parameter callback to update matrices when parameters change
  // The generate_parameter_library handles this internally, but we need to 
  // register our own callback for when parameters are actually updated
  parameter_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& parameters) {
      // Let the auto-generated library validate the parameters
      auto result = param_listener_->update(parameters);
      if (result.successful) {
        // Get the updated parameters and reconfigure matrices
        params_ = param_listener_->get_params();
        this->update_admittance_parameters();
        RCLCPP_INFO(get_logger(), "Parameters updated - admittance matrices reconfigured");
      }
      return result;
    });
  
  // Initialize robot state vectors with appropriate sizes (Drake notation: q, q_dot)
  const auto joint_count = params_.joints.size();
  q_current_.resize(joint_count, 0.0);            // Current sensor positions
  q_dot_current_.resize(joint_count, 0.0);        // Current sensor velocities
  // q_cmd_ removed - velocity controller doesn't need position integration
  q_dot_cmd_.resize(joint_count, 0.0);            // Computed command velocities
  
  // Initialize 6-DOF admittance control matrices (xyz + rpy)
  M_ = Matrix6d::Identity();           // Virtual mass matrix
  D_ = Matrix6d::Identity();           // Damping matrix  
  K_ = Matrix6d::Zero();               // Stiffness matrix (pure admittance by default)
  
  // Initialize 6-DOF control state vectors to zero
  Wrench_tcp_base_ = Vector6d::Zero();            // External forces/torques (filtered)
  V_tcp_base_commanded_ = Vector6d::Zero();       // Commanded Cartesian velocity
  V_tcp_base_desired_ = Vector6d::Zero();         // Internal velocity state
  
  // Initialize pose representations
  X_tcp_base_current_ = Eigen::Isometry3d::Identity();  // Current TCP pose
  X_tcp_base_desired_ = Eigen::Isometry3d::Identity();  // Target reference pose
  
  // Set up TF2 transform system for coordinate frame management
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Subscribe to filtered force/torque data from wrench_node
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench_tcp_base",
      rclcpp::SensorDataQoS(),
      std::bind(&AdmittanceNode::wrench_callback, this, std::placeholders::_1));
  // Subscribe to robot joint state feedback
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      10,
      std::bind(&AdmittanceNode::joint_state_callback, this, std::placeholders::_1));
  // Subscribe to desired pose updates
  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/admittance_node/desired_pose",
      10,
      std::bind(&AdmittanceNode::desired_pose_callback, this, std::placeholders::_1));
  // Subscribe to robot description from robot_state_publisher
  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&AdmittanceNode::robot_description_callback, this, std::placeholders::_1));
  // Publish joint velocity commands to UR controller
  velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 1);
  // Try to initialize kinematics (will succeed if robot_description is already published)
  if (!load_kinematics()) {
    RCLCPP_WARN(get_logger(), "Waiting for robot_description to be published...");
  }
  // Configure all admittance matrices from parameters
  update_admittance_parameters();

  // Pre-allocate velocity message to avoid real-time memory allocation
  velocity_msg_.data.resize(params_.joints.size());
  
  // Initialize workspace and velocity limits (TODO: load from parameters)
  workspace_limits_ << -1.0, 1.0,   // X limits [m]
                       -1.0, 1.0,   // Y limits [m]
                       0.0, 1.5;    // Z limits [m]
  arm_max_vel_ = 0.5;  // Maximum Cartesian velocity [m/s]
  arm_max_acc_ = 1.0;  // Maximum Cartesian acceleration [m/s^2]
  admittance_ratio_ = 1.0;  // Full admittance by default
  
  // Don't initialize desired pose here - wait for robot to be fully loaded
  
  RCLCPP_INFO(get_logger(), "UR Admittance Controller node created");
}

// ROS2 handles automatic cleanup of timers and subscriptions

// Initialize the admittance controller - must be called before control loop
bool AdmittanceNode::initialize() {
  RCLCPP_INFO(get_logger(), "=== Starting Admittance Controller Initialization ===");
  
  // Step 1: Wait for robot to be ready
  wait_for_robot_ready();
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "Shutdown requested during robot initialization");
    return false;
  }
  
  // Step 2: Initialize kinematics
  wait_for_kinematics();
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "Shutdown requested during kinematics initialization");
    return false;
  }
  
  // Step 3: Wait for transforms
  wait_for_transformations();
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "Shutdown requested while waiting for transforms");
    return false;
  }
  
  // Step 4: Initialize reference pose
  wait_for_initial_pose();
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "Shutdown requested during pose initialization");
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "=== Initialization Complete - Ready for Control ===");
  RCLCPP_INFO(get_logger(), "UR Admittance Controller ready - push the robot to move it!");
  return true;
}

// Main control cycle - called from main loop at 100Hz
void AdmittanceNode::control_cycle() {
  // Compute forward kinematics from joint positions (industry standard approach)
  computeForwardKinematics();
  
  // Compute admittance dynamics
  if (!compute_admittance()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Admittance computation failed");
    return;
  }
  
  // Apply safety limits
  limit_to_workspace();
  limit_joint_velocities();
  
  // Send velocity commands to robot
  send_commands_to_robot();
}

// Receive filtered force/torque sensor data
void AdmittanceNode::wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  current_wrench_ = *msg;

  // Extract 6-DOF wrench data (forces + torques)
  // Data is already filtered and in base frame from wrench_node
  Wrench_tcp_base_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
}

// Update robot joint state from sensor feedback
void AdmittanceNode::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  // Mark that we're receiving joint states (robot is loaded)
  if (!joint_states_received_) {
    joint_states_received_ = true;
    RCLCPP_INFO(get_logger(), "Joint states received - robot is loaded in simulation");
  }

  // Map joint positions and velocities by name to handle different joint ordering
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), params_.joints[i]);
    if (it != msg->name.end()) {
      const size_t idx = std::distance(msg->name.begin(), it);
      
      // Update position
      if (idx < msg->position.size()) {
        q_current_[i] = msg->position[idx];  // Store current sensor position

        // Velocity controller doesn't need position initialization
      }
      
      // Update velocity if available
      if (idx < msg->velocity.size()) {
        q_dot_current_[i] = msg->velocity[idx];  // Store current sensor velocity
      }
    }
  }
  
  // Set flag for FK computation
  joint_states_updated_ = true;
}

// Callback for robot description updates from robot_state_publisher
void AdmittanceNode::robot_description_callback(const std_msgs::msg::String::ConstSharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Received robot description from topic");
  robot_description_ = msg->data;
  
  // Initialize kinematics if not already done
  if (!kinematics_initialized_ && load_kinematics()) {
    kinematics_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Kinematics successfully initialized from robot description");
  }
}

// Initialize KDL kinematics from URDF
bool AdmittanceNode::load_kinematics() {
  // Use cached URDF from robot_description topic subscription
  std::string urdf_string = robot_description_;
  
  if (urdf_string.empty()) {
    RCLCPP_WARN(get_logger(), "robot_description not yet received from topic");
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

    RCLCPP_INFO(get_logger(), "KDL kinematics ready: %s -> %s (%d joints, %d segments)",
                params_.base_link.c_str(), params_.tip_link.c_str(),
                kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfSegments());
    RCLCPP_INFO(get_logger(), "Forward kinematics solver initialized (industry standard direct computation)");

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Kinematics initialization failed: %s", e.what());
    return false;
  }
}


// Set reference pose to current robot position (zero initial error)
bool AdmittanceNode::initialize_desired_pose() {
  if (desired_pose_initialized_) {
    return true;  // Already initialized
  }

  // Get current end-effector pose from transforms
  Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
  
  // Try to get valid transform with timeout
  try {
    const auto transform = tf_buffer_->lookupTransform(
        params_.base_link,
        params_.tip_link,
        tf2::TimePointZero,
        std::chrono::seconds(1));  // Wait up to 1 second
    current_pose = tf2::transformToEigen(transform);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(), "Cannot initialize desired pose yet - TF not ready: %s", ex.what());
    return false;  // Try again later
  }

  // Set reference pose to current pose (ensures zero initial error)
  X_tcp_base_desired_ = current_pose;
  desired_pose_initialized_ = true;

  const auto& pos = current_pose.translation();
  RCLCPP_INFO(get_logger(), "Reference pose initialized at [%.3f, %.3f, %.3f] m",
              pos.x(), pos.y(), pos.z());

  return true;
}

// Handle desired pose updates from ROS2 topic
void AdmittanceNode::desired_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  // Convert geometry_msgs::Pose to Eigen::Isometry3d
  Eigen::Isometry3d new_desired_pose = Eigen::Isometry3d::Identity();
  
  // Set translation
  new_desired_pose.translation() << msg->pose.position.x,
                                    msg->pose.position.y,
                                    msg->pose.position.z;
  
  // Set rotation from quaternion
  Eigen::Quaterniond q(msg->pose.orientation.w,
                       msg->pose.orientation.x,
                       msg->pose.orientation.y,
                       msg->pose.orientation.z);
  q.normalize();
  new_desired_pose.linear() = q.toRotationMatrix();
  
  // Update desired pose (no mutex needed - single threaded)
  X_tcp_base_desired_ = new_desired_pose;
  
  RCLCPP_INFO(get_logger(), "Desired pose updated to [%.3f, %.3f, %.3f]",
              new_desired_pose.translation().x(),
              new_desired_pose.translation().y(),
              new_desired_pose.translation().z());
}

// Send velocity commands to robot
void AdmittanceNode::send_commands_to_robot() {
  // Convert Cartesian velocity to joint velocities
  if (!compute_joint_velocities(V_tcp_base_commanded_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "IK velocity solver failed");
    return;
  }
  
  // Publish joint velocity commands
  for (size_t i = 0; i < velocity_msg_.data.size(); ++i) {
    velocity_msg_.data[i] = q_dot_cmd_[i];
  }
  velocity_pub_->publish(velocity_msg_);
}

// Apply workspace limits to Cartesian motion
void AdmittanceNode::limit_to_workspace() {
  // TODO: Implement workspace boundary checking
  // Check if current position is within workspace_limits_
  // workspace_limits_ = [x_min, x_max, y_min, y_max, z_min, z_max]
  
  // For now, just pass through
  // Future implementation will check boundaries and limit velocities
}

// Apply velocity limits to joint commands
void AdmittanceNode::limit_joint_velocities() {
  // TODO: Implement joint velocity saturation
  // Check if joint velocities exceed arm_max_vel_
  // Scale down proportionally if needed
  
  // For now, just pass through
  // Future implementation will apply velocity limits
}

// Apply admittance ratio to scale force response
void AdmittanceNode::apply_admittance_ratio(double ratio) {
  // TODO: Scale external wrench by admittance ratio
  // Wrench_tcp_base_ = ratio * Wrench_tcp_base_raw_
  // Allows variable compliance (0 = no response, 1 = full response)
  
  admittance_ratio_ = std::max(0.0, std::min(1.0, ratio));
}

// Transform utility functions (ROS1 style)
bool AdmittanceNode::get_transform_matrix(Eigen::Isometry3d& transform,
                                         const std::string& from_frame,
                                         const std::string& to_frame,
                                         const std::chrono::milliseconds& timeout) {
  try {
    const auto tf_stamped = tf_buffer_->lookupTransform(
        from_frame, to_frame,
        tf2::TimePointZero,
        timeout
    );
    transform = tf2::transformToEigen(tf_stamped);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "Transform lookup failed (%s → %s): %s",
                        from_frame.c_str(), to_frame.c_str(), ex.what());
    return false;
  }
}

// Get 6x6 rotation matrix for wrench transformation (ROS1 pattern)
bool AdmittanceNode::get_rotation_matrix_6d(Matrix6d& rotation_matrix,
                                           const std::string& from_frame,
                                           const std::string& to_frame) {
  Eigen::Isometry3d transform;
  if (get_transform_matrix(transform, from_frame, to_frame)) {
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = transform.rotation();
    rotation_matrix.bottomRightCorner(3, 3) = transform.rotation();
    return true;
  }
  return false;
}

// Wait for robot to be ready (joint states available)
void AdmittanceNode::wait_for_robot_ready() {
  RCLCPP_INFO(get_logger(), "Waiting for robot to be ready...");
  
  while (!joint_states_received_ && rclcpp::ok()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for joint states...");
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  RCLCPP_INFO(get_logger(), "Joint states received - robot is ready");
}

// Wait for kinematics to be initialized
void AdmittanceNode::wait_for_kinematics() {
  RCLCPP_INFO(get_logger(), "Waiting for kinematics initialization...");
  
  while (!kinematics_initialized_ && rclcpp::ok()) {
    if (load_kinematics()) {
      kinematics_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Kinematics initialized successfully");
      break;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for robot_description...");
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// Wait for initial pose to be set
void AdmittanceNode::wait_for_initial_pose() {
  RCLCPP_INFO(get_logger(), "Initializing reference pose...");
  
  while (!desired_pose_initialized_ && rclcpp::ok()) {
    if (initialize_desired_pose()) {
      RCLCPP_INFO(get_logger(), "Reference pose initialized successfully");
      break;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for valid transform to initialize pose...");
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// Wait for all required transforms to be available
void AdmittanceNode::wait_for_transformations() {
  RCLCPP_INFO(get_logger(), "Waiting for required transforms...");
  
  // Wait for base->tip transform (most important)
  while (!tf_buffer_->canTransform(params_.base_link, params_.tip_link, 
                                   tf2::TimePointZero) && rclcpp::ok()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for transform: %s → %s", 
                         params_.base_link.c_str(), params_.tip_link.c_str());
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  RCLCPP_INFO(get_logger(), "All transforms ready");
}

// Publish arm state in world frame (for visualization/debugging)
void AdmittanceNode::publish_arm_state_in_world() {
  // TODO: Implement publishing of end-effector pose/twist in world frame
  // Similar to ROS1's publish_arm_state_in_world()
  // This would publish geometry_msgs::PoseStamped and TwistStamped
}

// Publish debugging signals (forces, equilibrium, etc.)
void AdmittanceNode::publish_debugging_signals() {
  // TODO: Implement publishing of debug information
  // - External wrench
  // - Control wrench  
  // - Equilibrium position
  // - Admittance ratio
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