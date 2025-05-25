/**
 * @file controller_setup.cpp
 * @brief Lifecycle management and configuration for UR Admittance Controller
 * 
 * This file contains all one-time setup operations including parameter
 * loading, kinematic configuration, and ROS 2 lifecycle management.
 */

#include "admittance_controller.hpp"

// Additional includes needed for setup
#include <algorithm>
#include <rclcpp/callback_group.hpp>

namespace ur_admittance_controller {

AdmittanceController::AdmittanceController()
: controller_interface::ChainableControllerInterface(),
  rt_logger_(rclcpp::get_logger("admittance_controller").get_child("realtime"))
{
}

controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // CHAINABLE MODE: We don't claim hardware command interfaces
  // We only export reference interfaces via export_reference_interfaces()
  return config;  // Empty - we don't write to hardware directly
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint : params_.joints) {
    for (const auto & interface : params_.state_interfaces) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  
  // Add F/T sensor interfaces if available
  if (!params_.ft_sensor_name.empty()) {
    static const char* axes[] = {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
    for (const auto & axis : axes) {
      config.names.push_back(params_.ft_sensor_name + "/" + axis);
    }
  }
  
  return config;
}

std::vector<hardware_interface::CommandInterface> AdmittanceController::on_export_reference_interfaces()
{
  // In ROS 2 Jazzy, we implement this method to provide reference interfaces to downstream controllers
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    // Export each joint position reference for downstream controllers
    reference_interfaces.emplace_back(
      params_.joints[i], "position", &joint_position_references_[i]);
  }
  
  return reference_interfaces;
}

controller_interface::CallbackReturn AdmittanceController::on_init()
{
  try {
    param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize control matrices
    mass_ = Matrix6d::Identity();
    mass_inverse_ = mass_.inverse();  // Pre-compute inverse for performance
    damping_ = Matrix6d::Identity();
    stiffness_ = Matrix6d::Zero();
    wrench_ = wrench_filtered_ = Vector6d::Zero();
    pose_error_ = velocity_error_ = Vector6d::Zero();
    desired_accel_ = desired_vel_ = Vector6d::Zero();
    cart_twist_ = Vector6d::Zero();
    
    // Initialize pose tracking for impedance control
    desired_pose_ = Eigen::Isometry3d::Identity();
    current_pose_ = Eigen::Isometry3d::Identity();
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  
  // Validate mass parameters
  for (size_t i = 0; i < 6; ++i) {
    if (params_.admittance.mass[i] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), 
        "Invalid mass[%zu] = %f. Must be positive.", i, params_.admittance.mass[i]);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Validate damping ratios
  for (size_t i = 0; i < 6; ++i) {
    if (params_.admittance.damping_ratio[i] < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), 
        "Invalid damping_ratio[%zu] = %f. Must be non-negative.", 
        i, params_.admittance.damping_ratio[i]);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Validate velocity limits
  if (params_.max_linear_velocity <= 0.0 || params_.max_angular_velocity <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), 
      "Invalid velocity limits. Must be positive.");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Validate filter coefficient
  if (params_.admittance.filter_coefficient < 0.0 || params_.admittance.filter_coefficient > 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(), 
      "Invalid filter_coefficient = %f. Must be in [0.0, 1.0]", 
      params_.admittance.filter_coefficient);
    return controller_interface::CallbackReturn::ERROR;
  }
  
  if (!loadKinematics()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load kinematics");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Build admittance matrices
  for (size_t i = 0; i < 6; ++i) {
    mass_(i, i) = params_.admittance.mass[i];
    stiffness_(i, i) = params_.admittance.stiffness[i];
    
    // Smooth damping calculation to avoid discontinuity when stiffness changes
    // from zero to non-zero values
    const double stiffness_threshold = 1.0; // N/m or Nm/rad threshold for blending
    double stiffness_value = params_.admittance.stiffness[i];
    
    if (stiffness_value <= 0.0) {
      // Pure admittance mode - direct damping value in Ns/m or Nms/rad
      // Base damping scaled by mass for consistent units
      damping_(i, i) = params_.admittance.damping_ratio[i] * 
        std::sqrt(params_.admittance.mass[i]); // Consistent units Ns/m or Nms/rad
    } 
    else if (stiffness_value >= stiffness_threshold) {
      // Full impedance mode - critical damping formula
      damping_(i, i) = 2.0 * params_.admittance.damping_ratio[i] * 
        std::sqrt(params_.admittance.mass[i] * stiffness_value);
    }
    else {
      // Smooth transition zone - blend between the two formulas
      double blend_factor = stiffness_value / stiffness_threshold; // 0.0 to 1.0
      
      // Calculate both damping values
      double admittance_damping = params_.admittance.damping_ratio[i] * 
        std::sqrt(params_.admittance.mass[i]);
        
      double impedance_damping = 2.0 * params_.admittance.damping_ratio[i] * 
        std::sqrt(params_.admittance.mass[i] * stiffness_value);
      
      // Smoothly blend between the two values
      damping_(i, i) = (1.0 - blend_factor) * admittance_damping + 
                       blend_factor * impedance_damping;
    }
  }
  
  // Load real joint limits from URDF
  if (!loadJointLimitsFromURDF(get_node(), params_.joints, joint_limits_)) {
    RCLCPP_WARN(get_node()->get_logger(), 
      "Failed to load joint limits from URDF, using parameter/default limits");
    
    joint_limits_.resize(params_.joints.size());
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      joint_limits_[i] = {-6.283, 6.283, 3.14, 10.0};
      
      if (i < params_.joint_limits_position_min.size()) {
        joint_limits_[i].min_position = params_.joint_limits_position_min[i];
      }
      if (i < params_.joint_limits_position_max.size()) {
        joint_limits_[i].max_position = params_.joint_limits_position_max[i];
      }
      if (i < params_.joint_limits_velocity_max.size()) {
        joint_limits_[i].max_velocity = params_.joint_limits_velocity_max[i];
      }
    }
  }
  
  // Load safe startup parameters
  try {
    // Load parameters from the config file
    // Since these aren't in the auto-generated parameter schema, we need to declare them first
    get_node()->declare_parameter("startup.trajectory_duration", safe_startup_params_.trajectory_duration);
    get_node()->declare_parameter("startup.stiffness_ramp_time", safe_startup_params_.stiffness_ramp_time);
    get_node()->declare_parameter("startup.max_position_error", safe_startup_params_.max_position_error);
    get_node()->declare_parameter("startup.max_orientation_error", safe_startup_params_.max_orientation_error);
    
    // Now retrieve the parameters
    safe_startup_params_.trajectory_duration = 
      get_node()->get_parameter("startup.trajectory_duration").as_double();
    safe_startup_params_.stiffness_ramp_time = 
      get_node()->get_parameter("startup.stiffness_ramp_time").as_double();
    safe_startup_params_.max_position_error = 
      get_node()->get_parameter("startup.max_position_error").as_double();
    safe_startup_params_.max_orientation_error = 
      get_node()->get_parameter("startup.max_orientation_error").as_double();
    
    RCLCPP_INFO(get_node()->get_logger(), "Safe startup parameters loaded:");
    RCLCPP_INFO(get_node()->get_logger(), "  Trajectory duration: %.2f s", 
                safe_startup_params_.trajectory_duration);
    RCLCPP_INFO(get_node()->get_logger(), "  Stiffness ramp time: %.2f s", 
                safe_startup_params_.stiffness_ramp_time);
    RCLCPP_INFO(get_node()->get_logger(), "  Max position error: %.3f m", 
                safe_startup_params_.max_position_error);
    RCLCPP_INFO(get_node()->get_logger(), "  Max orientation error: %.3f rad", 
                safe_startup_params_.max_orientation_error);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_node()->get_logger(), 
      "Failed to load some safe startup parameters, using defaults: %s", e.what());
  }
  
  // Create standard publishers first and store them in member variables
  cart_vel_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    "~/cartesian_velocity_command", rclcpp::SystemDefaultsQoS());
  
  // Trajectory publisher removed - using reference interfaces is sufficient for controller chaining
    
  // Debug publisher for pose error
  pose_error_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    "~/pose_error", rclcpp::SystemDefaultsQoS());
    
  // Initialize real-time safe publishers with the standard publishers
  rt_cart_vel_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
    cart_vel_pub_);
  
  // Realtime trajectory publisher removed - using reference interfaces is sufficient for controller chaining
    
  rt_pose_error_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
    pose_error_pub_);
    
  // Add services and subscriptions for impedance mode control
  reset_pose_service_holder_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/reset_desired_pose",
    [weak_this = std::weak_ptr<AdmittanceController>(shared_from_this())]
    (const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (auto controller = weak_this.lock()) {
        controller->handle_reset_pose(request, response);
      }
    });
  reset_pose_service_ = reset_pose_service_holder_;

  // Create subscriber for setting the desired pose
  set_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/set_desired_pose", 10,
    std::bind(&AdmittanceController::handle_set_desired_pose, this,
              std::placeholders::_1));
              
  // Create publishers for monitoring - store in both working and holder variables
  current_pose_pub_holder_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/current_pose", 10);
  current_pose_pub_ = current_pose_pub_holder_;
    
  desired_pose_pub_holder_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/desired_pose", 10);
  desired_pose_pub_ = desired_pose_pub_holder_;
  
  // Add safe movement service for impedance mode
  move_to_pose_service_holder_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/move_to_start_pose",
    [weak_this = std::weak_ptr<AdmittanceController>(shared_from_this())]
    (const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (auto controller = weak_this.lock()) {
        controller->handle_move_to_pose(request, response);
      }
    });
  move_to_pose_service_ = move_to_pose_service_holder_;
              
  RCLCPP_INFO(get_node()->get_logger(), "Impedance mode communication setup complete");
  
  // Initialize all vectors with proper sizes for real-time safety
  joint_positions_.resize(params_.joints.size());
  joint_position_references_.resize(params_.joints.size());
  
  // Pre-allocate vectors to avoid dynamic memory allocation in the control loop
  current_pos_.resize(params_.joints.size());
  joint_deltas_.resize(params_.joints.size());
  cart_displacement_deltas_.resize(6);
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!waitForTransforms()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get transforms");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Reset all integrator states
  pose_error_.setZero();
  velocity_error_.setZero(); 
  desired_accel_.setZero();
  desired_vel_.setZero();
  cart_twist_.setZero();
  wrench_.setZero();
  
  // Initialize pose tracking with current robot pose
  desired_pose_ = Eigen::Isometry3d::Identity();
  current_pose_ = Eigen::Isometry3d::Identity();
  
  // When activating, set desired pose to current pose to avoid initial jumps
  // Use our real-time safe transform cache instead of direct TF lookups
  if (ee_transform_cache_.isValid()) {
    // The transform cache should already be initialized by waitForTransforms()
    current_pose_ = tf2::transformToEigen(ee_transform_cache_.cache.transform);
    desired_pose_ = current_pose_;
    RCLCPP_INFO(get_node()->get_logger(), "Initialized desired pose from cached transform");
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "No valid transform cache for initial pose - using identity");
    // Use identity as fallback
    current_pose_ = Eigen::Isometry3d::Identity();
    desired_pose_ = current_pose_;
  }
  wrench_filtered_.setZero();
  
  // Cache interface indices for RT performance
  cacheInterfaceIndices();
  
  // Initialize joint positions from state
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_positions_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
    joint_position_references_[i] = joint_positions_[i];
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "AdmittanceController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Clear ALL integrator states to avoid jerk on restart
  desired_vel_.setZero();
  pose_error_.setZero();
  cart_twist_.setZero();
  wrench_filtered_.setZero();
  desired_accel_.setZero();
  wrench_.setZero();
  
  // Note: For tf_listener, we don't fully reset it during deactivation
  // as that would lose configuration, but we do want to minimize its
  // activity while the controller is inactive. Full cleanup happens in on_cleanup().
  
  RCLCPP_INFO(get_node()->get_logger(), "AdmittanceController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Starting cleanup...");
  
  // Clean up ROS services to break circular references
  reset_pose_service_holder_.reset();
  move_to_pose_service_holder_.reset();
  // Service weak_ptrs will be automatically invalidated
  
  // Clean up subscribers and publishers
  set_pose_sub_.reset();
  current_pose_pub_.reset();
  desired_pose_pub_.reset();
  
  // Clean up real-time publishers
  rt_cart_vel_pub_.reset();
  rt_pose_error_pub_.reset();
  
  // Clean up transform-related resources
  RCLCPP_INFO(get_node()->get_logger(), "Cleaning up transform listeners...");
  
  // Stop transform listener first (it has a thread)
  tf_listener_.reset();
  
  // Then clear the buffer
  tf_buffer_.reset();
  
  // Clear transform caches
  ft_transform_cache_.reset();
  ee_transform_cache_.reset();
  
  // Clean up parameter buffer
  param_buffer_.writeFromNonRT(ur_admittance_controller::Params());
  
  // Reset kinematic resources
  kinematics_.reset();
  
  RCLCPP_INFO(get_node()->get_logger(), "Cleanup complete");
  return controller_interface::CallbackReturn::SUCCESS;
}

void AdmittanceController::cacheInterfaceIndices()
{
  // Cache state interface indices
  pos_state_indices_.resize(params_.joints.size());
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto name = params_.joints[i] + "/position";
    auto it = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
      [name](const auto & iface) { return iface.get_name() == name; });
    pos_state_indices_[i] = std::distance(state_interfaces_.cbegin(), it);
  }
  
  // Cache F/T interface indices
  ft_indices_.resize(DOF, -1);
  if (!params_.ft_sensor_name.empty()) {
    static constexpr std::array<const char*, DOF> axes = {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
    
    for (size_t i = 0; i < axes.size(); ++i) {
      const auto name = params_.ft_sensor_name + "/" + axes[i];
      auto it = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&name](const auto & iface) { return iface.get_name() == name; });
      if (it != state_interfaces_.cend()) {
        ft_indices_[i] = std::distance(state_interfaces_.cbegin(), it);
      }
    }
  }
}

bool AdmittanceController::loadKinematics()
{
  try {
    // Store the class loader in a member variable so it doesn't get destroyed
    kinematics_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      params_.kinematics_plugin_package, "kinematics_interface::KinematicsInterface");
    
    // Get the plugin instance with the correct unique_ptr type
    auto plugin_instance = kinematics_loader_->createUniqueInstance(params_.kinematics_plugin_name);
    kinematics_ = std::move(plugin_instance);
    
    if (!kinematics_.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create kinematics instance");
      return false;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "Loaded kinematics: %s", 
      params_.kinematics_plugin_name.c_str());
    return true;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception loading kinematics: %s", e.what());
    return false;
  }
}

bool AdmittanceController::waitForTransforms()
{
  const auto timeout = rclcpp::Duration::from_seconds(5.0);
  std::string error;
  auto now = get_node()->get_clock()->now();
  
  // First check if transforms are available
  bool transforms_available = 
    tf_buffer_->canTransform(params_.world_frame, params_.base_link, 
                            rclcpp::Time(0), timeout, &error) &&
    tf_buffer_->canTransform(params_.base_link, params_.tip_link, 
                            rclcpp::Time(0), timeout, &error) &&
    tf_buffer_->canTransform(params_.base_link, params_.ft_frame, 
                            rclcpp::Time(0), timeout, &error);
  
  if (!transforms_available) {
    RCLCPP_ERROR(get_node()->get_logger(), "Required transforms not available: %s", error.c_str());
    return false;
  }
  
  // Initialize transform cache frame names and reset the cache
  ft_transform_cache_.reset();
  ee_transform_cache_.reset();
  
  // Set frame names for later lookups
  ft_transform_cache_.target_frame = params_.base_link;
  ft_transform_cache_.source_frame = params_.ft_frame;
  
  ee_transform_cache_.target_frame = params_.base_link;
  ee_transform_cache_.source_frame = params_.tip_link;
  
  // Explicitly request the first transform update
  transform_update_needed_.store(true);
  
  // Do an initial transform update to populate the caches
  try {
    updateTransformCaches();
    
    // Verify the transforms were cached properly
    if (!ft_transform_cache_.isValid() || !ee_transform_cache_.isValid()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize transform caches");
      return false;
    }
    
    return true;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during transform cache initialization: %s", ex.what());
    return false;
  }
}

bool AdmittanceController::loadJointLimitsFromURDF(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const std::vector<std::string> & joint_names,
  std::vector<JointLimits> & limits)
{
  try {
    // Get robot description parameter
    std::string robot_description;
    if (!node->get_parameter("robot_description", robot_description)) {
      RCLCPP_ERROR(node->get_logger(), 
        "Failed to get robot_description parameter");
      return false;
    }
    
    // Parse URDF
    urdf::Model model;
    if (!model.initString(robot_description)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF");
      return false;
    }
    
    limits.resize(joint_names.size());
    
    // Extract limits for each joint
    for (size_t i = 0; i < joint_names.size(); ++i) {
      auto joint = model.getJoint(joint_names[i]);
      if (!joint) {
        RCLCPP_ERROR(node->get_logger(), 
          "Joint '%s' not found in URDF", joint_names[i].c_str());
        return false;
      }
      
      if (!joint->limits) {
        RCLCPP_ERROR(node->get_logger(), 
          "No limits defined for joint '%s'", joint_names[i].c_str());
        return false;
      }
      
      // CRITICAL: Use real UR limits from URDF
      limits[i].min_position = joint->limits->lower;
      limits[i].max_position = joint->limits->upper;
      limits[i].max_velocity = joint->limits->velocity;
      limits[i].max_acceleration = joint->limits->effort / 10.0; // Rough estimate
      
      RCLCPP_INFO(node->get_logger(),
        "Joint %s limits: pos[%.3f, %.3f], vel[%.3f]",
        joint_names[i].c_str(),
        limits[i].min_position,
        limits[i].max_position, 
        limits[i].max_velocity);
    }
    
    return true;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), 
      "Exception loading joint limits: %s", e.what());
    return false;
  }
}

} // namespace ur_admittance_controller
