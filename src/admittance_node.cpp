#include "admittance_node.hpp"
#include <chrono>

namespace ur_admittance_controller {

AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions & options)
: Node("admittance_node", options)
{
  RCLCPP_INFO(get_logger(), "Initializing Admittance Node...");
  
  // Initialize parameter listener
  param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
    this->get_node_parameters_interface());
  params_ = param_listener_->get_params();
  
  // Resize state vectors
  joint_positions_.resize(params_.joints.size(), 0.0);
  joint_velocities_.resize(params_.joints.size(), 0.0);
  joint_position_references_.resize(params_.joints.size(), 0.0);
  current_pos_.resize(params_.joints.size(), 0.0);
  joint_deltas_.resize(params_.joints.size(), 0.0);
  cart_displacement_deltas_.resize(6, 0.0);
  
  // Initialize control matrices
  mass_ = Matrix6d::Identity();
  damping_ = Matrix6d::Identity();
  stiffness_ = Matrix6d::Zero();
  
  // Initialize control variables
  F_sensor_base_ = Vector6d::Zero();
  V_base_tip_base_ = Vector6d::Zero();
  desired_vel_ = Vector6d::Zero();
  desired_accel_ = Vector6d::Zero();
  
  // Initialize poses
  X_base_tip_current_ = Eigen::Isometry3d::Identity();
  X_base_tip_desired_ = Eigen::Isometry3d::Identity();
  
  // Create transform infrastructure
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Create subscriptions
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/wrist_ft_sensor", 
    rclcpp::SensorDataQoS(),
    std::bind(&AdmittanceNode::wrenchCallback, this, std::placeholders::_1));
    
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    std::bind(&AdmittanceNode::jointStateCallback, this, std::placeholders::_1));
    
  // Subscribe to robot description from ur_simulation_gz
  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
    "/robot_description",
    rclcpp::QoS(1).transient_local(),
    std::bind(&AdmittanceNode::robotDescriptionCallback, this, std::placeholders::_1));
  
  // Create publisher
  trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/scaled_joint_trajectory_controller/joint_trajectory", 1);
  
  // Initialize monitoring publishers
  cart_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/admittance_cartesian_velocity", 10);
  pose_error_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/admittance_pose_error", 10);
  
  // Initialize transforms and kinematics
  if (!initializeTransforms()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize transforms");
  }
  
  if (!loadKinematics()) {
    RCLCPP_ERROR(get_logger(), "Failed to load kinematics");
  }
  
  // Initialize matrices from parameters
  updateMassMatrix();
  updateDampingMatrix();
  updateStiffnessMatrix();
  
  // Pre-allocate messages
  trajectory_msg_.joint_names = params_.joints;
  trajectory_msg_.points.resize(1);
  trajectory_msg_.points[0].positions.resize(params_.joints.size());
  trajectory_msg_.points[0].velocities.resize(params_.joints.size());
  // Reduced trajectory timing for smoother motion (10-20ms instead of 100ms)
  trajectory_msg_.points[0].time_from_start = rclcpp::Duration::from_seconds(0.02);
  
  // Start dedicated control thread for high-frequency admittance control
  RCLCPP_INFO(get_logger(), "Starting dedicated control thread for admittance control");
  running_.store(true);
  control_thread_ = std::thread(&AdmittanceNode::controlThreadFunction, this);
    
  RCLCPP_INFO(get_logger(), "Admittance Node initialized successfully");
}

AdmittanceNode::~AdmittanceNode()
{
  // Stop control thread if running
  if (running_.load()) {
    running_.store(false);
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
  }
}

void AdmittanceNode::controlThreadFunction()
{
  // Fixed control rate for admittance control (500 Hz = 2ms period)
  // This provides excellent performance while being CPU-efficient
  constexpr double CONTROL_RATE_HZ = 500.0;
  const auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / CONTROL_RATE_HZ));
  
  RCLCPP_INFO(get_logger(), "Control thread running at %.0f Hz", CONTROL_RATE_HZ);
  
  auto next_time = std::chrono::steady_clock::now();
  
  while (rclcpp::ok() && running_.load()) {
    // Run admittance control computation
    controlLoop();
    
    // Maintain fixed rate with high precision
    next_time += period_ns;
    std::this_thread::sleep_until(next_time);
  }
  
  RCLCPP_INFO(get_logger(), "Control thread stopped");
}

void AdmittanceNode::wrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  current_wrench_ = *msg;
  
  // Extract wrench data
  Vector6d raw_wrench;
  raw_wrench(0) = msg->wrench.force.x;
  raw_wrench(1) = msg->wrench.force.y;
  raw_wrench(2) = msg->wrench.force.z;
  raw_wrench(3) = msg->wrench.torque.x;
  raw_wrench(4) = msg->wrench.torque.y;
  raw_wrench(5) = msg->wrench.torque.z;
  
  // Transform to base frame using direct tf2 lookup
  F_sensor_base_ = transformWrench(raw_wrench);
  
  // Apply low-pass filter
  wrench_filtered_ = params_.admittance.filter_coefficient * F_sensor_base_ + 
    (1.0 - params_.admittance.filter_coefficient) * wrench_filtered_;
}

void AdmittanceNode::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  
  // Update joint positions based on configured joint names
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), params_.joints[i]);
    if (it != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it);
      if (idx < msg->position.size()) {
        joint_positions_[i] = msg->position[idx];
      }
    }
  }
}

void AdmittanceNode::robotDescriptionCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(robot_description_mutex_);
  robot_description_ = msg->data;
  robot_description_received_.store(true);
  
  RCLCPP_INFO(get_logger(), "Received robot description from ur_simulation_gz (length: %zu)", 
    robot_description_.length());
}

void AdmittanceNode::controlLoop()
{
  // Calculate control period
  static rclcpp::Time last_time = now();
  rclcpp::Time current_time = now();
  rclcpp::Duration period = current_time - last_time;
  
  // Skip if period is invalid
  if (period.seconds() <= 0.0 || period.seconds() > 0.1) {
    return;
  }
  
  // Update last_time only after successful period validation
  last_time = current_time;
  
  // Run admittance control computation
  if (!computeAdmittanceStep(period)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, 
      "Admittance computation failed");
    safeStop();
    // Reset last_time on computation failure to maintain timing consistency
    last_time = current_time;
    return;
  }
  
  // Update pre-allocated trajectory message
  trajectory_msg_.header.stamp = current_time;
  
  // Update trajectory point data
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    trajectory_msg_.points[0].positions = joint_positions_;
  }
  trajectory_msg_.points[0].velocities = joint_velocities_;
  
  trajectory_pub_->publish(trajectory_msg_);
  
  // Publish monitoring data
  publishMonitoringData();
}

bool AdmittanceNode::initializeTransforms()
{
  // Just wait for transforms to become available
  return waitForTransforms();
}

bool AdmittanceNode::loadJointLimitsFromURDF()
{
  // Check if robot description has been received
  if (!robot_description_received_.load()) {
    RCLCPP_WARN(get_logger(), "Robot description not yet received from /robot_description topic");
    return false;
  }
  
  std::string urdf_string;
  {
    std::lock_guard<std::mutex> lock(robot_description_mutex_);
    urdf_string = robot_description_;
  }
  
  if (urdf_string.empty()) {
    RCLCPP_ERROR(get_logger(), "Robot description is empty");
    return false;
  }
  
  // Parse URDF
  urdf::Model model;
  if (!model.initString(urdf_string)) {
    RCLCPP_ERROR(get_logger(), "Failed to parse robot description URDF");
    return false;
  }
  
  joint_limits_.resize(params_.joints.size());
  
  // Load official UR5e limits from URDF
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    auto joint = model.getJoint(params_.joints[i]);
    if (!joint) {
      RCLCPP_ERROR(get_logger(), 
        "Joint '%s' not found in robot description URDF", params_.joints[i].c_str());
      return false;
    }
    
    if (!joint->limits) {
      RCLCPP_ERROR(get_logger(), 
        "No limits defined for joint '%s' in URDF", params_.joints[i].c_str());
      return false;
    }
    
    // Extract official UR5e limits
    joint_limits_[i].min_position = joint->limits->lower;
    joint_limits_[i].max_position = joint->limits->upper;
    joint_limits_[i].max_velocity = joint->limits->velocity;
    
    // UR5e has no published acceleration limits (confirmed by official ur_description)
    // Acceleration smoothing is handled by scaled_joint_trajectory_controller
    
    RCLCPP_INFO(get_logger(),
      "Joint %s limits from official UR5e URDF: pos[%.3f, %.3f], vel[%.3f]",
      params_.joints[i].c_str(),
      joint_limits_[i].min_position,
      joint_limits_[i].max_position, 
      joint_limits_[i].max_velocity);
  }
  
  RCLCPP_INFO(get_logger(), "Successfully loaded joint limits from official UR5e robot description");
  return true;
}

// Fallback method with temporary hardcoded limits
bool AdmittanceNode::loadJointLimitsFromParameters()
{
  joint_limits_.resize(params_.joints.size());
  
  // Fallback hardcoded UR5e limits
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    // Default UR5e limits
    joint_limits_[i].min_position = -6.283;  // ±360°
    joint_limits_[i].max_position = 6.283;
    joint_limits_[i].max_velocity = 3.14;    // 180°/s
    
    // Special case for elbow joint (limited to ±180° for planning)
    if (params_.joints[i] == "elbow_joint") {
      joint_limits_[i].min_position = -3.142;  // ±180°
      joint_limits_[i].max_position = 3.142;
    }
  }
  
  RCLCPP_WARN(get_logger(), "Using fallback joint limits - robot description not available");
  return true;
}

bool AdmittanceNode::loadKinematics()
{
  try {
    kinematics_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      params_.kinematics_plugin_package, "kinematics_interface::KinematicsInterface");
    
    auto plugin_instance = kinematics_loader_->createUniqueInstance(params_.kinematics_plugin_name);
    
    // Check if plugin creation succeeded before assigning
    if (!plugin_instance) {
      RCLCPP_ERROR(get_logger(), "Failed to create kinematics instance: plugin_instance is null");
      return false;
    }
    
    // Convert the custom deleter unique_ptr to standard unique_ptr
    kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(plugin_instance.release());
    
    RCLCPP_INFO(get_logger(), "Loaded kinematics: %s", 
      params_.kinematics_plugin_name.c_str());
    
    // Load joint limits from robot_description topic first, fallback to hardcoded
    if (!loadJointLimitsFromURDF()) {
      RCLCPP_WARN(get_logger(), "Could not load joint limits from robot_description, using fallback");
      if (!loadJointLimitsFromParameters()) {
        RCLCPP_ERROR(get_logger(), "Failed to load joint limits");
        return false;
      }
    }
    
    return true;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception loading kinematics: %s", e.what());
    return false;
  }
}

bool AdmittanceNode::waitForTransforms()
{
  RCLCPP_INFO(get_logger(), "Waiting for transforms...");
  
  // Wait for F/T transform if needed
  if (params_.ft_frame != params_.base_link) {
    if (!tf_buffer_->canTransform(params_.base_link, params_.ft_frame, 
                                  tf2::TimePointZero)) {
      RCLCPP_WARN(get_logger(), "Transform %s -> %s not yet available", 
                   params_.ft_frame.c_str(), params_.base_link.c_str());
      return false;
    }
  }
  
  // Wait for end-effector transform
  if (!tf_buffer_->canTransform(params_.base_link, params_.tip_link,
                                tf2::TimePointZero)) {
    RCLCPP_WARN(get_logger(), "Transform %s -> %s not yet available",
                 params_.tip_link.c_str(), params_.base_link.c_str());
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "All required transforms are available");
  return true;
}

}  // namespace ur_admittance_controller

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}