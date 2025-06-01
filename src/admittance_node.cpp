#include "admittance_node.hpp"
#include <chrono>

namespace ur_admittance_controller {

// Removed unused isValidJointIndex() function

AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions & options)
: Node("admittance_node", options)
{
  RCLCPP_INFO(get_logger(), "Initializing Admittance Node...");
  
  // Initialize parameter listener
  param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
    this->get_node_parameters_interface());
  params_ = param_listener_->get_params();
  
  // Resize state vectors - RACE CONDITION FIX
  joint_positions_.resize(params_.joints.size(), 0.0);      // Sensor data
  joint_positions_cmd_.resize(params_.joints.size(), 0.0);  // Command data  
  joint_velocities_.resize(params_.joints.size(), 0.0);
  current_pos_.resize(params_.joints.size(), 0.0);
  
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
  
  
  
  // Note: Transform and kinematics initialization will happen lazily 
  // in the control loop once robot_description and tf data are available
  
  // Initialize matrices from parameters
  updateMassMatrix();
  updateDampingMatrix();
  
  // Parameter updates handled automatically by generate_parameter_library
  for (size_t i = 0; i < 6; ++i) {
    stiffness_(i, i) = params_.admittance.stiffness[i];
  }
  
  // Pre-allocate messages
  trajectory_msg_.joint_names = params_.joints;
  trajectory_msg_.points.resize(1);
  trajectory_msg_.points[0].positions.resize(params_.joints.size());
  trajectory_msg_.points[0].velocities.resize(params_.joints.size());
  // Match control loop timing: 500Hz = 2ms period
  trajectory_msg_.points[0].time_from_start = rclcpp::Duration::from_seconds(0.002);
  
  // No more thread communication overhead
  
  // Start unified control thread - maximum speed, no artificial rate limiting
  RCLCPP_INFO(get_logger(), "Starting unified control thread at MAXIMUM SPEED");
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

// Removed wasteful timer callback - merged into unified control thread

void AdmittanceNode::controlThreadFunction()
{
  RCLCPP_INFO(get_logger(), "Unified control thread running at MAXIMUM SPEED (no rate limiting)");
  
  auto last_time = std::chrono::steady_clock::now();
  
  while (rclcpp::ok() && running_.load()) {
    auto current_time = std::chrono::steady_clock::now();
    auto period_ns = current_time - last_time;
    double dt = period_ns.count() * 1e-9;  // Convert to seconds
    
    // Run at maximum speed, but skip if time step is too small (prevent excessive CPU usage)
    if (dt >= 0.0001) {  // Min 0.1ms = 10kHz max theoretical rate
      if (unifiedControlStep(dt)) {
        last_time = current_time;
      }
    }
    
    // Brief yield to prevent complete CPU starvation of other threads
    std::this_thread::yield();
  }
  
  RCLCPP_INFO(get_logger(), "Unified control thread stopped");
}

void AdmittanceNode::wrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  current_wrench_ = *msg;
  
  // Extract wrench data
  Vector6d raw_wrench;
  raw_wrench << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  
  // Transform to base frame using direct tf2 lookup
  F_sensor_base_ = transformWrench(raw_wrench);
  
  // Apply EMA filter - validation handled by generate_parameter_library
  wrench_filtered_ = params_.admittance.filter_coefficient * F_sensor_base_ + 
    (1.0 - params_.admittance.filter_coefficient) * wrench_filtered_;
}

void AdmittanceNode::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  
  // Update sensor joint positions based on configured joint names
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), params_.joints[i]);
    if (it != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it);
      if (idx < msg->position.size()) {
        joint_positions_[i] = msg->position[idx];  // Sensor data only
        
        // Initialize command positions to sensor positions on first callback
        if (joint_positions_cmd_[i] == 0.0) {
          joint_positions_cmd_[i] = msg->position[idx];
        }
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

// Removed separate integration function - merged into unified control step



bool AdmittanceNode::loadKinematics()
{
  // Wait for robot description
  if (!robot_description_received_.load()) {
    RCLCPP_WARN(get_logger(), "Robot description not available for kinematics setup");
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

  try {
    // Parse URDF into KDL tree (direct KDL approach like tutorial)
    if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF into KDL tree");
      return false;
    }

    // Extract kinematic chain from base to tip
    if (!kdl_tree_.getChain(params_.base_link, params_.tip_link, kdl_chain_)) {
      RCLCPP_ERROR(get_logger(), "Failed to extract chain from %s to %s", 
        params_.base_link.c_str(), params_.tip_link.c_str());
      return false;
    }

    // Create inverse kinematics velocity solver with WDLS (Weighted Damped Least Squares)
    const double eps = 0.00001;  // Precision threshold
    const int maxiter = 150;     // Maximum iterations
    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_, eps, maxiter);

    // Configure WDLS parameters for better performance
    const double lambda = 0.01;  // Damping factor for singularity robustness
    ik_vel_solver_->setLambda(lambda);
    
    // Note: No joint limits enforced - pure mathematical framework

    kinematics_ready_ = true;
    
    RCLCPP_INFO(get_logger(), "KDL WDLS kinematics initialized successfully");
    RCLCPP_INFO(get_logger(), "Chain: %s -> %s (%d joints, %d segments)",
      params_.base_link.c_str(), params_.tip_link.c_str(),
      kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfSegments());
    RCLCPP_INFO(get_logger(), "WDLS solver configured for singularity robustness (lambda=%.3f)", lambda);
    
    return true;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "KDL initialization failed: %s", e.what());
    return false;
  }
}


bool AdmittanceNode::initializeDesiredPose()
{
  if (desired_pose_initialized_.load()) {
    return true;  // Already initialized
  }
  
  // Get current robot pose
  Eigen::Isometry3d current_pose;
  if (!getCurrentEndEffectorPose(current_pose)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
      "Cannot initialize desired pose - current pose not available");
    return false;
  }
  
  // Set desired pose to current pose (zero error)
  {
    std::lock_guard<std::mutex> lock(desired_pose_mutex_);
    X_base_tip_desired_ = current_pose;
  }
  
  desired_pose_initialized_.store(true);
  
  RCLCPP_INFO(get_logger(), "Desired pose initialized to current robot pose");
  RCLCPP_INFO(get_logger(), "Position: [%.3f, %.3f, %.3f]", 
    current_pose.translation().x(), 
    current_pose.translation().y(), 
    current_pose.translation().z());
  
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