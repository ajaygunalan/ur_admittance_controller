#include "admittance_node.hpp"
#include <chrono>

namespace ur_admittance_controller {

// Main constructor - initializes admittance control system

AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  RCLCPP_INFO(get_logger(), "Initializing UR Admittance Controller - 6-DOF Force-Compliant Motion Control");
  
  // Set up dynamic parameter system with auto-generated parameter library
  param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
      this->get_node_parameters_interface());
  params_ = param_listener_->get_params();
  
  // Initialize robot state vectors with appropriate sizes (Drake notation: q, v)
  const auto joint_count = params_.joints.size();
  q_.resize(joint_count, 0.0);                    // Current sensor positions
  q_cmd_.resize(joint_count, 0.0);                // Integrated command positions
  v_.resize(joint_count, 0.0);                    // Computed velocities
  q_current_.resize(joint_count, 0.0);            // Working copy for kinematics
  
  // Initialize 6-DOF admittance control matrices (xyz + rpy)
  M_ = Matrix6d::Identity();           // Virtual mass matrix
  D_ = Matrix6d::Identity();           // Damping matrix  
  K_ = Matrix6d::Zero();               // Stiffness matrix (pure admittance by default)
  
  // Initialize 6-DOF control state vectors to zero
  F_sensor_base_ = Vector6d::Zero();        // External forces/torques
  V_base_tip_base_ = Vector6d::Zero();      // Commanded Cartesian velocity
  V_base_tip_desired_ = Vector6d::Zero();   // Integrated velocity
  A_base_tip_desired_ = Vector6d::Zero();   // Computed acceleration
  
  // Initialize pose representations
  X_base_tip_current_ = Eigen::Isometry3d::Identity();  // Current end-effector pose
  X_base_tip_desired_ = Eigen::Isometry3d::Identity();  // Target reference pose
  
  // Set up TF2 transform system for coordinate frame management
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Subscribe to force/torque sensor data with high-priority QoS
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrist_ft_sensor",
      rclcpp::SensorDataQoS(),
      std::bind(&AdmittanceNode::WrenchCallback, this, std::placeholders::_1));
  // Subscribe to robot joint state feedback
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      10,
      std::bind(&AdmittanceNode::JointStateCallback, this, std::placeholders::_1));
  // Subscribe to URDF robot model with persistent QoS
  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).transient_local(),
      std::bind(&AdmittanceNode::RobotDescriptionCallback, this, std::placeholders::_1));
  // Publish joint trajectory commands to UR controller
  trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/scaled_joint_trajectory_controller/joint_trajectory", 1);
  // Note: Kinematics setup occurs lazily when robot_description is received
  // Configure admittance matrices from parameters
  UpdateMassMatrix();
  UpdateDampingMatrix();
  // Set up stiffness matrix from parameters (6-DOF: xyz + rpy)
  for (size_t i = 0; i < 6; ++i) {
    K_(i, i) = params_.admittance.stiffness[i];
  }
  // Pre-allocate trajectory message to avoid real-time memory allocation
  trajectory_msg_.joint_names = params_.joints;
  trajectory_msg_.points.resize(1);
  trajectory_msg_.points[0].positions.resize(joint_count);
  trajectory_msg_.points[0].velocities.resize(joint_count);
  trajectory_msg_.points[0].time_from_start = 
      rclcpp::Duration::from_seconds(constants::MIN_CONTROL_PERIOD_SEC);
  // Start high-frequency control timer using ROS2 standard approach
  RCLCPP_INFO(get_logger(), "Starting admittance control at %.0f Hz",
              constants::TARGET_CONTROL_RATE_HZ);
  control_timer_ = create_wall_timer(
      std::chrono::duration<double>(constants::MIN_CONTROL_PERIOD_SEC),
      std::bind(&AdmittanceNode::ControlTimerCallback, this));
  RCLCPP_INFO(get_logger(), "UR Admittance Controller ready - push the robot to move it!");
}

// ROS2 handles automatic cleanup of timers and subscriptions

// High-frequency control timer callback (500Hz) - main admittance control loop
void AdmittanceNode::ControlTimerCallback() {
  // Compute precise time step for this control iteration
  static auto last_time = std::chrono::steady_clock::now();
  const auto current_time = std::chrono::steady_clock::now();
  const auto period_ns = current_time - last_time;
  const double dt = period_ns.count() * 1e-9;  // Convert nanoseconds to seconds

  // Execute main admittance control algorithm
  if (UnifiedControlStep(dt)) {
    last_time = current_time;
  }
}

// Process incoming force/torque sensor data with filtering and coordinate transforms
void AdmittanceNode::WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  current_wrench_ = *msg;

  // Extract 6-DOF wrench data (forces + torques)
  Vector6d raw_wrench;
  raw_wrench << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

  // Transform wrench from sensor frame to robot base frame
  F_sensor_base_ = TransformWrench(raw_wrench);

  // Apply exponential moving average filter to reduce sensor noise
  const double alpha = params_.admittance.filter_coefficient;
  F_sensor_filtered_ = alpha * F_sensor_base_ + (1.0 - alpha) * F_sensor_filtered_;
}

// Update robot joint state from sensor feedback with thread-safe access
void AdmittanceNode::JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);

  // Map joint positions by name to handle different joint ordering
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), params_.joints[i]);
    if (it != msg->name.end()) {
      const size_t idx = std::distance(msg->name.begin(), it);
      if (idx < msg->position.size()) {
        q_[i] = msg->position[idx];  // Store current sensor position

        // Initialize command positions on first callback to avoid jumps
        if (q_cmd_[i] == 0.0) {
          q_cmd_[i] = msg->position[idx];
        }
      }
    }
  }
}

// Process URDF robot model for kinematics setup
void AdmittanceNode::RobotDescriptionCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(robot_description_mutex_);
  robot_description_ = msg->data;
  robot_description_received_.store(true);

  RCLCPP_INFO(get_logger(), "Received URDF robot description (%zu bytes) - kinematics ready",
              robot_description_.length());
}



// Initialize KDL kinematics from URDF for inverse velocity solving
bool AdmittanceNode::LoadKinematics() {
  // Ensure robot description is available
  if (!robot_description_received_.load()) {
    RCLCPP_WARN(get_logger(), "Waiting for robot description to setup kinematics");
    return false;
  }

  std::string urdf_string;
  {
    std::lock_guard<std::mutex> lock(robot_description_mutex_);
    urdf_string = robot_description_;
  }

  if (urdf_string.empty()) {
    RCLCPP_ERROR(get_logger(), "Empty robot description - cannot setup kinematics");
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

    kinematics_ready_ = true;

    RCLCPP_INFO(get_logger(), "KDL kinematics ready: %s -> %s (%d joints, %d segments)",
                params_.base_link.c_str(), params_.tip_link.c_str(),
                kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfSegments());

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Kinematics initialization failed: %s", e.what());
    return false;
  }
}


// Set reference pose to current robot position (zero initial error)
bool AdmittanceNode::InitializeDesiredPose() {
  if (desired_pose_initialized_.load()) {
    return true;  // Already initialized
  }

  // Get current end-effector pose from transforms
  Eigen::Isometry3d current_pose;
  if (!GetCurrentEndEffectorPose(current_pose)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Waiting for transforms to initialize reference pose");
    return false;
  }

  // Set reference pose to current pose (ensures zero initial error)
  {
    std::lock_guard<std::mutex> lock(desired_pose_mutex_);
    X_base_tip_desired_ = current_pose;
  }

  desired_pose_initialized_.store(true);

  const auto& pos = current_pose.translation();
  RCLCPP_INFO(get_logger(), "Reference pose initialized at [%.3f, %.3f, %.3f] m",
              pos.x(), pos.y(), pos.z());

  return true;
}


}  // namespace ur_admittance_controller

// Main entry point - initialize ROS2 and start admittance control node
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();

  RCLCPP_INFO(node->get_logger(), "Starting UR Admittance Controller - Ready for force interaction!");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}