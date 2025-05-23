#include "admittance_controller.hpp"

namespace ur_admittance_controller
{

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
    
    if (params_.admittance.stiffness[i] > 0.0) {
      damping_(i, i) = 2.0 * params_.admittance.damping_ratio[i] * 
        std::sqrt(params_.admittance.mass[i] * params_.admittance.stiffness[i]);
    } else {
      damping_(i, i) = params_.admittance.damping_ratio[i];
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
  
  // Create standard publishers first
  auto cart_vel_pub = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    "~/cartesian_velocity_command", rclcpp::SystemDefaultsQoS());
  
  auto trajectory_pub = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/scaled_joint_trajectory_controller/joint_trajectory", 
    rclcpp::QoS(1).best_effort().durability_volatile());
    
  // Initialize real-time safe publishers with the standard publishers
  rt_cart_vel_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
    cart_vel_pub);
  
  rt_trajectory_pub_ = std::make_unique<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>(
    trajectory_pub);
  
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
  try {
    auto tf = tf_buffer_->lookupTransform(params_.base_link, params_.tip_link, 
                                         tf2::TimePointZero, tf2::durationFromSec(TRANSFORM_TIMEOUT));
    desired_pose_ = tf2::transformToEigen(tf);
    current_pose_ = desired_pose_;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to get initial pose: %s", ex.what());
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
  
  RCLCPP_INFO(get_node()->get_logger(), "AdmittanceController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Update parameters if dynamic
  if (params_.dynamic_parameters) {
    params_ = param_listener_->get_params();
  }
  
  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  try {
    // Parameter hot-reload for live tuning
    if (params_.dynamic_parameters) {
      auto new_params = param_listener_->get_params();
      
      // Check which parameters have changed
      bool mass_changed = (params_.admittance.mass != new_params.admittance.mass);
      bool stiffness_changed = (params_.admittance.stiffness != new_params.admittance.stiffness);
      bool damping_changed = (params_.admittance.damping_ratio != new_params.admittance.damping_ratio);
      
      // Only update matrices if control parameters changed
      if (mass_changed || stiffness_changed || damping_changed) {
        // Always update params first
        params_ = new_params;
        
        // Update mass matrix if mass changed
        if (mass_changed) {
          for (size_t i = 0; i < 6; ++i) {
            mass_(i, i) = params_.admittance.mass[i];
          }
          // Pre-compute mass inverse only when mass changes
          mass_inverse_ = mass_.inverse();
          RCLCPP_INFO(get_node()->get_logger(), "Mass parameters updated");
        }
        
        // Update stiffness matrix if stiffness changed
        if (stiffness_changed) {
          for (size_t i = 0; i < 6; ++i) {
            stiffness_(i, i) = params_.admittance.stiffness[i];
          }
          RCLCPP_INFO(get_node()->get_logger(), "Stiffness parameters updated");
        }
        
        // Update damping matrix if stiffness or damping ratio changed
        // (since damping depends on both stiffness and damping ratio)
        if (stiffness_changed || damping_changed) {
          for (size_t i = 0; i < 6; ++i) {
            if (params_.admittance.stiffness[i] > 0.0) {
              // Impedance mode damping formula: D = 2ζ√(MK)
              damping_(i, i) = 2.0 * params_.admittance.damping_ratio[i] * 
                std::sqrt(params_.admittance.mass[i] * params_.admittance.stiffness[i]);
            } else {
              // Pure admittance mode: direct damping coefficient
              damping_(i, i) = params_.admittance.damping_ratio[i];
            }
          }
          RCLCPP_INFO(get_node()->get_logger(), "Damping parameters updated");
        }
      } else {
        // Update other non-control parameters without matrix recalculation
        params_ = new_params;
      }
    }
    
    // Read F/T data using cached indices (RT-optimized)
    Vector6d raw_wrench;
    for (size_t i = 0; i < 6; ++i) {
      if (ft_indices_[i] >= 0) {
        raw_wrench(i) = state_interfaces_[ft_indices_[i]].get_optional().value();
      } else {
        raw_wrench(i) = 0.0;
      }
    }
    
    // Transform wrench from base to tool frame with RT-safe approach using cache
    auto now = get_node()->get_clock()->now();
    
    // Check cache validity (100ms timeout)
    if (!ft_transform_cache_.valid || 
        (now - ft_transform_cache_.last_update).seconds() > TRANSFORM_TIMEOUT) {
      try {
        // Use zero timeout for real-time safety (non-blocking) but with consistent timestamp
        ft_transform_cache_.transform = tf_buffer_->lookupTransform(
          params_.ft_frame, params_.base_link, 
          tf2_ros::fromMsg(now), tf2::durationFromSec(0.0));
          
        // Compute adjoint matrix once and cache it
        Eigen::Matrix3d R = tf2::transformToEigen(ft_transform_cache_.transform).rotation();
        ft_transform_cache_.adjoint = Matrix6d::Zero();
        ft_transform_cache_.adjoint.block<3, 3>(0, 0) = R;
        ft_transform_cache_.adjoint.block<3, 3>(3, 3) = R;
        ft_transform_cache_.valid = true;
        ft_transform_cache_.last_update = now;
      } catch (const tf2::TransformException & ex) {
        // Use existing cache if available
        if (!ft_transform_cache_.valid) {
          // No valid cache yet
          if (rclcpp::ok()) {
            RCLCPP_WARN_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
              "Transform from %s to %s not available: %s", 
              params_.ft_frame.c_str(), params_.base_link.c_str(), ex.what());
          }
          wrench_ = raw_wrench;  // Use untransformed wrench
          return controller_interface::return_type::OK;
        }
      }
    }
    
    // Update current_pose_ for impedance control
    if (!ee_transform_cache_.valid || 
        (now - ee_transform_cache_.last_update).seconds() > TRANSFORM_TIMEOUT) {
      try {
        ee_transform_cache_.transform = tf_buffer_->lookupTransform(
          params_.base_link, params_.tip_link, 
          tf2_ros::fromMsg(now), tf2::durationFromSec(0.0));
        current_pose_ = tf2::transformToEigen(ee_transform_cache_.transform);
        ee_transform_cache_.valid = true;
        ee_transform_cache_.last_update = now;
      } catch (const tf2::TransformException &) {
        // Continue with previous pose if transform fails
      }
    }
    
    // Apply transform using cached adjoint
    wrench_ = ft_transform_cache_.adjoint.transpose() * raw_wrench;
    
    // Log occasionally when using a cached transform
    if ((now - ft_transform_cache_.last_update).seconds() > 0.5) {
      RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Using cached transform for real-time safety");
    }
    
    // Apply unified filtering
    wrench_filtered_ = params_.admittance.filter_coefficient * wrench_ + 
      (1.0 - params_.admittance.filter_coefficient) * wrench_filtered_;
    
    // Deadband check
    bool motion_above_threshold = false;
    for (size_t i = 0; i < 6; ++i) {
      if (std::abs(wrench_filtered_(i)) > params_.admittance.min_motion_threshold) {
        motion_above_threshold = true;
        break;
      }
    }
    
    if (!motion_above_threshold) {
      cart_twist_.setZero();
      for (size_t i = 0; i < params_.joints.size(); ++i) {
        joint_position_references_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
      }
      return controller_interface::return_type::OK;
    }
    
    // Compute pose error for impedance control
    pose_error_ = computePoseError();
    
    // Compute admittance with impedance: M*a + D*v + K*e = F_ext
    // Use pre-computed mass inverse for performance
    desired_accel_ = mass_inverse_ * 
      (wrench_filtered_ - damping_ * desired_vel_ - stiffness_ * pose_error_);
    desired_vel_ += desired_accel_ * period.seconds();
    
    // Apply axis enables
    for (size_t i = 0; i < 6; ++i) {
      if (!params_.admittance.enabled_axes[i]) {
        desired_vel_(i) = 0.0;
      }
    }
    
    // Apply Cartesian velocity safety limits
    // Separate linear and angular components
    double linear_velocity_magnitude = desired_vel_.head<3>().norm();
    double angular_velocity_magnitude = desired_vel_.tail<3>().norm();
    
    // Clamp linear velocity if needed
    if (linear_velocity_magnitude > params_.max_linear_velocity && linear_velocity_magnitude > 0) {
      desired_vel_.head<3>() *= (params_.max_linear_velocity / linear_velocity_magnitude);
    }
    
    // Clamp angular velocity if needed
    if (angular_velocity_magnitude > params_.max_angular_velocity && angular_velocity_magnitude > 0) {
      desired_vel_.tail<3>() *= (params_.max_angular_velocity / angular_velocity_magnitude);
    }
    
    cart_twist_ = desired_vel_;
    
    // DRIFT PREVENTION: Reset positions when nearly stationary to prevent accumulation of numerical errors
    // Use configurable threshold from parameters (default 0.001 = 1mm/s threshold)
    if (cart_twist_.norm() < params_.admittance.drift_reset_threshold) {
      // Reset integration when robot is nearly stationary
      desired_vel_.setZero();
      cart_twist_.setZero();
      
      // Reset to actual positions using C++17 range-based for loop
      size_t i = 0;
      for (const auto& index : pos_state_indices_) {
        joint_positions_[i++] = state_interfaces_[index].get_optional().value();
      }
      
      // Update desired pose to current pose to prevent drift
      desired_pose_ = current_pose_;
      
      RCLCPP_DEBUG(get_node()->get_logger(), "Drift correction applied - resetting to actual position");
    }
    
    // Convert to joint space using real kinematics
    // Use pre-allocated vector for real-time safety
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      current_pos_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
    }
    
    // CRITICAL FIX: Convert velocity to displacement deltas by multiplying by Δt
    // Use pre-allocated vector for real-time safety
    for (size_t i = 0; i < 6; ++i) {
      cart_displacement_deltas_[i] = cart_twist_(i) * period.seconds();
    }
    
    // Use pre-allocated joint_deltas_ vector for real-time safety
    
    if (kinematics_ && (*kinematics_)->convert_cartesian_deltas_to_joint_deltas(
          current_pos_, cart_displacement_deltas_, params_.tip_link, joint_deltas_)) {
      
      // Apply joint deltas and limits with proper ordering
      for (size_t i = 0; i < params_.joints.size() && i < joint_deltas_.size(); ++i) {
        // Start with base position + delta
        joint_positions_[i] = current_pos_[i] + joint_deltas_[i];
        
        // FIRST: Apply velocity limits
        double velocity = joint_deltas_[i] / period.seconds();
        if (std::abs(velocity) > joint_limits_[i].max_velocity) {
          // Scale down the delta to respect velocity limit
          double scale = joint_limits_[i].max_velocity / std::abs(velocity);
          joint_positions_[i] = current_pos_[i] + joint_deltas_[i] * scale;
        }
        
        // THEN: Apply position limits (always last!)
        joint_positions_[i] = std::clamp(
          joint_positions_[i], 
          joint_limits_[i].min_position, 
          joint_limits_[i].max_position);
      }
      
      // Write to REFERENCE interfaces (for chaining with ScaledJTC)
      for (size_t i = 0; i < params_.joints.size(); ++i) {
        joint_position_references_[i] = joint_positions_[i];  // ScaledJTC reads these
      }
      
      // Send trajectory message to scaled_joint_trajectory_controller
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.joint_names = params_.joints;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      
      // Set positions from calculated joint positions
      point.positions.resize(params_.joints.size());
      for (size_t i = 0; i < params_.joints.size(); ++i) {
        point.positions[i] = joint_positions_[i];
      }
      
      // Add velocities from joint deltas
      point.velocities.resize(params_.joints.size(), 0.0);
      for (size_t i = 0; i < params_.joints.size() && i < joint_deltas_.size(); ++i) {
        point.velocities[i] = joint_deltas_[i] / period.seconds();
      }
      
      // Set small future time from start
      point.time_from_start = rclcpp::Duration::from_seconds(TRANSFORM_TIMEOUT);
      
      // Add point and publish trajectory
      traj_msg.points.push_back(point);
      if (rt_trajectory_pub_->trylock()) {
        rt_trajectory_pub_->msg_ = traj_msg;
        rt_trajectory_pub_->unlockAndPublish();
      }
      
    } else {
      if (rclcpp::ok()) {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
          "Kinematics conversion failed - zeroing motion");
      }
      
      // ERROR HANDLING: Zero motion on kinematics failure
      cart_twist_.setZero();
      desired_vel_.setZero();
      
      // Maintain current position references
      for (size_t i = 0; i < params_.joints.size(); ++i) {
        joint_position_references_[i] = current_pos_[i];
      }
      
      // Send trajectory message with current positions (no motion)
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.joint_names = params_.joints;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      
      // Set positions from current joint positions
      point.positions.resize(params_.joints.size());
      for (size_t i = 0; i < params_.joints.size(); ++i) {
        point.positions[i] = current_pos_[i];
      }
      
      // Zero velocities
      point.velocities.resize(params_.joints.size(), 0.0);
      
      // Set small future time from start
      point.time_from_start = rclcpp::Duration::from_seconds(TRANSFORM_TIMEOUT);
      
      // Add point and publish trajectory
      traj_msg.points.push_back(point);
      if (rt_trajectory_pub_->trylock()) {
        rt_trajectory_pub_->msg_ = traj_msg;
        rt_trajectory_pub_->unlockAndPublish();
      }
    }
    
    // Publish monitoring data (RT-safe)
    publishCartesianVelocity();
    
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in update: %s", e.what());
    }
    
    // SAFE FALLBACK: Zero all motion on any exception
    cart_twist_.setZero();
    desired_vel_.setZero();
    
    return controller_interface::return_type::ERROR;
  }
  
  return controller_interface::return_type::OK;
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

void AdmittanceController::publishCartesianVelocity()
{
  // Use realtime publisher with trylock pattern to ensure RT safety
  if (rt_cart_vel_pub_->trylock()) {
    // Access message via msg_ member
    auto& msg = rt_cart_vel_pub_->msg_;
    msg.linear.x = cart_twist_(0);
    msg.linear.y = cart_twist_(1);
    msg.linear.z = cart_twist_(2);
    msg.angular.x = cart_twist_(3);
    msg.angular.y = cart_twist_(4);
    msg.angular.z = cart_twist_(5);
    // Unlock and publish in a non-blocking way
    rt_cart_vel_pub_->unlockAndPublish();
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

Vector6d AdmittanceController::computePoseError()
{
  Vector6d error = Vector6d::Zero();
  
  // Position error (translation difference)
  error.head<3>() = desired_pose_.translation() - current_pose_.translation();
  
  // Orientation error (using angle-axis representation)
  Eigen::Matrix3d R_error = desired_pose_.rotation() * current_pose_.rotation().transpose();
  Eigen::AngleAxisd aa(R_error);
  error.tail<3>() = aa.angle() * aa.axis();
  
  return error;
}

bool AdmittanceController::waitForTransforms()
{
  const auto timeout = rclcpp::Duration::from_seconds(5.0);
  std::string error;
  
  return tf_buffer_->canTransform(params_.world_frame, params_.base_link, 
                                   rclcpp::Time(0), timeout, &error) &&
         tf_buffer_->canTransform(params_.base_link, params_.tip_link, 
                                   rclcpp::Time(0), timeout, &error) &&
         tf_buffer_->canTransform(params_.base_link, params_.ft_frame, 
                                   rclcpp::Time(0), timeout, &error);
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

}  // namespace ur_admittance_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ur_admittance_controller::AdmittanceController, 
  controller_interface::ChainableControllerInterface)