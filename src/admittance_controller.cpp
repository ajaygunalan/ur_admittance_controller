#include "admittance_controller.hpp"

namespace ur_admittance_controller
{

AdmittanceController::AdmittanceController()
: controller_interface::ChainableControllerInterface()
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
    
    // Initialize matrices and state using clean Eigen types
    mass_ = damping_ = stiffness_ = Matrix6d::Zero();
    wrench_ = wrench_filtered_ = Vector6d::Zero();
    pose_error_ = velocity_error_ = Vector6d::Zero();
    desired_accel_ = desired_vel_ = Vector6d::Zero();
    cart_twist_ = Vector6d::Zero();
    
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
  
  if (!loadKinematics()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load kinematics");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Build admittance matrices
  for (size_t i = 0; i < 6; ++i) {
    mass_(i, i) = params_.mass[i];
    stiffness_(i, i) = params_.stiffness[i];
    
    if (params_.stiffness[i] > 0.0) {
      damping_(i, i) = 2.0 * params_.damping_ratio[i] * 
        std::sqrt(params_.mass[i] * params_.stiffness[i]);
    } else {
      damping_(i, i) = params_.damping_ratio[i];
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
  
  // RT-safe publisher for monitoring
  cart_vel_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    "~/cartesian_velocity_command",
    rclcpp::QoS(1).best_effort().durability_volatile());
  
  joint_positions_.resize(params_.joints.size());
  joint_position_references_.resize(params_.joints.size());
  
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
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
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
      if (params_.mass != new_params.mass || 
          params_.stiffness != new_params.stiffness || 
          params_.damping_ratio != new_params.damping_ratio) {
        
        params_ = new_params;
        for (size_t i = 0; i < 6; ++i) {
          mass_(i, i) = params_.mass[i];
          stiffness_(i, i) = params_.stiffness[i];
          
          if (params_.stiffness[i] > 0.0) {
            damping_(i, i) = 2.0 * params_.damping_ratio[i] * 
              std::sqrt(params_.mass[i] * params_.stiffness[i]);
          } else {
            damping_(i, i) = params_.damping_ratio[i];
          }
        }
        RCLCPP_INFO(get_node()->get_logger(), "Updated admittance parameters");
      } else {
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
    
    // Transform wrench from base to tool frame
    try {
      auto tf = tf_buffer_->lookupTransform(params_.ft_frame, params_.base_link, tf2::TimePointZero, tf2::durationFromSec(0.1));
      Eigen::Matrix3d R = tf2::transformToEigen(tf).rotation();
      Matrix6d Ad = Matrix6d::Zero();
      Ad.block<3, 3>(0, 0) = R;
      Ad.block<3, 3>(3, 3) = R;
      wrench_ = Ad.transpose() * raw_wrench;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "TF transform failed: %s. Using untransformed wrench.", ex.what());
      wrench_ = raw_wrench;
    }
    
    // Apply unified filtering
    wrench_filtered_ = params_.filter_coefficient * wrench_ + 
      (1.0 - params_.filter_coefficient) * wrench_filtered_;
    
    // Deadband check
    bool motion_above_threshold = false;
    for (size_t i = 0; i < 6; ++i) {
      if (std::abs(wrench_filtered_(i)) > params_.min_motion_threshold) {
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
    
    // Compute admittance: M*a + D*v + K*e = F_ext
    // NOTE: pose_error_ remains zero for PURE ADMITTANCE CONTROL
    desired_accel_ = mass_.inverse() * 
      (wrench_filtered_ - damping_ * desired_vel_ - stiffness_ * pose_error_);
    desired_vel_ += desired_accel_ * period.seconds();
    
    // Apply axis enables
    for (size_t i = 0; i < 6; ++i) {
      if (!params_.admittance_enabled_axes[i]) {
        desired_vel_(i) = 0.0;
      }
    }
    cart_twist_ = desired_vel_;
    
    // Convert to joint space using real kinematics
    std::vector<double> current_pos(params_.joints.size());
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      current_pos[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
    }
    
    // CRITICAL FIX: Convert velocity to displacement deltas by multiplying by Δt
    std::vector<double> cart_displacement_deltas(6);
    for (size_t i = 0; i < 6; ++i) {
      cart_displacement_deltas[i] = cart_twist_(i) * period.seconds();
    }
    
    std::vector<double> joint_deltas;
    
    if (kinematics_ && kinematics_->convert_cartesian_deltas_to_joint_deltas(
          current_pos, cart_displacement_deltas, params_.tip_link, joint_deltas)) {
      
      // Apply joint deltas and limits
      for (size_t i = 0; i < params_.joints.size() && i < joint_deltas.size(); ++i) {
        joint_positions_[i] = current_pos[i] + joint_deltas[i];
        
        // CRITICAL: Clamp to real UR joint limits
        joint_positions_[i] = std::clamp(
          joint_positions_[i], 
          joint_limits_[i].min_position, 
          joint_limits_[i].max_position);
        
        // Also check velocity limits (delta/dt gives velocity)
        double velocity = joint_deltas[i] / period.seconds();
        if (std::abs(velocity) > joint_limits_[i].max_velocity) {
          // Scale down the delta to respect velocity limit
          double scale = joint_limits_[i].max_velocity / std::abs(velocity);
          joint_positions_[i] = current_pos[i] + joint_deltas[i] * scale;
        }
      }
      
      // Write to REFERENCE interfaces (for chaining with ScaledJTC)
      for (size_t i = 0; i < params_.joints.size(); ++i) {
        joint_position_references_[i] = joint_positions_[i];  // ScaledJTC reads these
      }
      
    } else {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Kinematics conversion failed - zeroing motion");
      
      // ERROR HANDLING: Zero motion on kinematics failure
      cart_twist_.setZero();
      desired_vel_.setZero();
      
      // Maintain current position references
      for (size_t i = 0; i < params_.joints.size(); ++i) {
        joint_position_references_[i] = current_pos[i];
      }
    }
    
    // Publish monitoring data (RT-safe)
    publishCartesianVelocity();
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Exception in update: %s", e.what());
    
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
  ft_indices_.resize(6, -1);
  if (!params_.ft_sensor_name.empty()) {
    static const char* axes[] = {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
    for (size_t i = 0; i < 6; ++i) {
      const auto name = params_.ft_sensor_name + "/" + axes[i];
      auto it = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
        [name](const auto & iface) { return iface.get_name() == name; });
      if (it != state_interfaces_.cend()) {
        ft_indices_[i] = std::distance(state_interfaces_.cbegin(), it);
      }
    }
  }
}

void AdmittanceController::publishCartesianVelocity()
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = cart_twist_(0);
  msg.linear.y = cart_twist_(1);
  msg.linear.z = cart_twist_(2);
  msg.angular.x = cart_twist_(3);
  msg.angular.y = cart_twist_(4);
  msg.angular.z = cart_twist_(5);
  cart_vel_pub_->publish(msg);
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
    
    if (!kinematics_) {
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