/**
 * @file admittance_controller.cpp
 * @brief Core implementation of the UR admittance controller
 *
 * This file contains the main lifecycle methods and interface configurations
 * for the admittance controller. It handles initialization, configuration,
 * activation, and cleanup of the controller resources.
 */

#include "admittance_controller.hpp"
#include "admittance_constants.hpp"
#include "matrix_utilities.hpp"

#include <algorithm>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rate.hpp>

namespace ur_admittance_controller {

using namespace constants;

AdmittanceController::AdmittanceController()
: controller_interface::ChainableControllerInterface(),
  rt_logger_(rclcpp::get_logger("admittance_controller").get_child("realtime"))
{
  // Initialize in constructor body if needed
}

controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Claim reference interfaces from downstream controller for proper chaining
  for (const auto & joint : params_.joints) {
    config.names.push_back(params_.downstream_controller_name + "/" + joint + "/position");
  }
  
  return config;
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Configure joint state interfaces
  for (const auto & joint : params_.joints) {
    for (const auto & interface : params_.state_interfaces) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  
  // Configure force/torque sensor interfaces
  // Note: We cannot check sensor mode here as params_ is not initialized yet
  // The controller will handle missing interfaces gracefully in on_configure
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
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  
  // Export joint position references for chaining
  // The prefix must be the controller name for proper chaining
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    reference_interfaces.emplace_back(
      get_node()->get_name(), params_.joints[i] + "/position", &reference_interfaces_[i]);
  }
  
  return reference_interfaces;
}

controller_interface::CallbackReturn AdmittanceController::on_init()
{
  try {
    // Initialize parameter listener
    param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
    
    // Resize reference interfaces storage for ChainableController
    // CRITICAL: This must match the number of exported interfaces
    reference_interfaces_.resize(params_.joints.size(), 0.0);
    
    // Initialize transform infrastructure
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize control matrices
    mass_ = Matrix6d::Identity();
    mass_inverse_ = mass_.inverse();
    damping_ = Matrix6d::Identity();
    stiffness_ = Matrix6d::Zero();
    
    // Initialize control variables
    F_sensor_base_ = wrench_filtered_ = Vector6d::Zero();
    error_tip_base_ = velocity_error_ = Vector6d::Zero();
    desired_accel_ = desired_vel_ = Vector6d::Zero();
    V_base_tip_base_ = Vector6d::Zero();
    
    // Initialize poses
    X_base_tip_desired_ = Eigen::Isometry3d::Identity();
    X_base_tip_current_ = Eigen::Isometry3d::Identity();
    
    // Initialize topic-based F/T sensor support
    // Pre-allocate buffers to avoid dynamic allocation
    for (auto& buffer : ft_buffer_storage_) {
      buffer.msg.header.frame_id.reserve(64);
      buffer.receive_time = get_node()->get_clock()->now();
      buffer.transform_valid = false;
    }
    
    // Initialize atomic variables
    ft_sequence_number_.store(0);
    last_processed_sequence_.store(0);
    consecutive_missed_updates_.store(0);
    
    // Initialize with first buffer
    rt_ft_buffer_.writeFromNonRT(ft_buffer_storage_[0]);
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // NOTE: Transform availability check moved to on_activate() to avoid segfault
  // tf_buffer_->canTransform() with timeout creates timers that can cause crashes
  // during lifecycle transitions (see geometry2 issue #460)
  
  // Force reload parameters to ensure latest values
  params_ = param_listener_->get_params();
  RCLCPP_INFO(get_node()->get_logger(), "Loaded parameters - ft_frame: %s, base_link: %s, tip_link: %s", 
    params_.ft_frame.c_str(), params_.base_link.c_str(), params_.tip_link.c_str());
  
  error_tip_base_.setZero();
  velocity_error_.setZero(); 
  desired_accel_.setZero();
  desired_vel_.setZero();
  V_base_tip_base_.setZero();
  F_sensor_base_.setZero();
  
  
  X_base_tip_desired_ = Eigen::Isometry3d::Identity();
  X_base_tip_current_ = Eigen::Isometry3d::Identity();
  
  if (transform_base_tip_.isValid()) {
    X_base_tip_current_ = tf2::transformToEigen(transform_base_tip_.getTransform().transform);
    X_base_tip_desired_ = X_base_tip_current_;
    RCLCPP_INFO(get_node()->get_logger(), "Initialized desired pose from cached transform");
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "No valid transform cache for initial pose - using identity");
    X_base_tip_current_ = Eigen::Isometry3d::Identity();
    X_base_tip_desired_ = X_base_tip_current_;
  }
  
  
  wrench_filtered_.setZero();
  
  // NOTE: cacheInterfaceIndices() moved to on_activate() where interfaces are available
  
  // Map command interfaces to joint indices
  if (!command_interfaces_.empty()) {
    cmd_interface_to_joint_index_.clear();
    cmd_interface_to_joint_index_.resize(command_interfaces_.size(), 0);
    
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      // For reference interfaces: prefix is controller name, interface name is "joint/position"
      const std::string& prefix_name = command_interfaces_[i].get_prefix_name();
      const std::string& interface_name = command_interfaces_[i].get_interface_name();
      
      // Extract joint name from "joint_name/position" format
      size_t slash_pos = interface_name.find('/');
      if (slash_pos == std::string::npos) {
        RCLCPP_ERROR(get_node()->get_logger(), 
          "Invalid interface name format: %s (expected 'joint_name/position')", 
          interface_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      
      std::string joint_name = interface_name.substr(0, slash_pos);
      std::string interface_type = interface_name.substr(slash_pos + 1);
      
      // Verify this is a position interface
      if (interface_type != "position") {
        RCLCPP_ERROR(get_node()->get_logger(), 
          "Expected position interface, got %s for joint %s", 
          interface_type.c_str(), joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      
      // Verify prefix matches downstream controller
      if (prefix_name != params_.downstream_controller_name) {
        RCLCPP_ERROR(get_node()->get_logger(), 
          "Interface prefix %s doesn't match downstream controller %s", 
          prefix_name.c_str(), params_.downstream_controller_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      
      auto it = std::find(params_.joints.begin(), params_.joints.end(), joint_name);
      if (it != params_.joints.end()) {
        size_t joint_idx = std::distance(params_.joints.begin(), it);
        cmd_interface_to_joint_index_[i] = joint_idx;
        RCLCPP_DEBUG(get_node()->get_logger(), 
          "Mapped reference interface %s/%s to joint index %zu", 
          prefix_name.c_str(), interface_name.c_str(), joint_idx);
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), 
          "Joint %s not found in controller joints", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }
  
  // Joint position initialization moved to on_activate() where state interfaces are available
  
  // Configure topic-based F/T sensor if needed
  use_topic_mode_ = (params_.sensor_interface.mode == "topic");
  
  if (use_topic_mode_) {
    RCLCPP_INFO(get_node()->get_logger(), "Configuring topic-based F/T sensor: %s", 
      params_.sensor_interface.topic_config.topic_name.c_str());
    
    // Create subscriber with SensorDataQoS
    ft_topic_sub_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      params_.sensor_interface.topic_config.topic_name,
      rclcpp::SensorDataQoS().keep_last(1).best_effort(),
      [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        this->ftSensorCallback(msg);
      }
    );
    
    // Pre-warm the buffers with zero wrench
    for (auto& buffer : ft_buffer_storage_) {
      buffer.msg.header.stamp = get_node()->get_clock()->now();
      buffer.msg.header.frame_id = params_.ft_frame;
      buffer.msg.wrench.force.x = 0.0;
      buffer.msg.wrench.force.y = 0.0;
      buffer.msg.wrench.force.z = 0.0;
      buffer.msg.wrench.torque.x = 0.0;
      buffer.msg.wrench.torque.y = 0.0;
      buffer.msg.wrench.torque.z = 0.0;
      buffer.receive_time = get_node()->get_clock()->now();
    }
    
    RCLCPP_INFO(get_node()->get_logger(), 
      "Topic mode enabled - F/T hardware interfaces will be ignored");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), 
      "Hardware mode enabled - using F/T sensor interfaces");
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "AdmittanceController configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating AdmittanceController...");
  
  // Wait for required transforms with retry loop (Nav2 pattern)
  const double transform_timeout_seconds = 5.0;
  const auto timeout_time = get_node()->get_clock()->now() + rclcpp::Duration::from_seconds(transform_timeout_seconds);
  rclcpp::Rate retry_rate(10);  // 10 Hz retry rate
  
  while (rclcpp::ok() && get_node()->get_clock()->now() < timeout_time) {
    if (waitForTransforms()) {
      RCLCPP_INFO(get_node()->get_logger(), "All required transforms available");
      break;
    }
    
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), 
      *get_node()->get_clock(), 1000,  // Log every 1 second
      "Waiting for transforms to become available... (%.1f seconds remaining)", 
      (timeout_time - get_node()->get_clock()->now()).seconds());
    
    retry_rate.sleep();
  }
  
  // Final check
  if (!waitForTransforms()) {
    RCLCPP_ERROR(get_node()->get_logger(), 
      "Failed to activate: Required transforms not available after %.1f seconds timeout",
      transform_timeout_seconds);
    return controller_interface::CallbackReturn::FAILURE;
  }
  
  // Reset control variables
  wrench_filtered_.setZero();
  F_sensor_base_.setZero();
  V_base_tip_base_.setZero();
  desired_vel_.setZero();
  desired_accel_.setZero();
  velocity_error_.setZero();
  error_tip_base_.setZero();
  
  // Reset joint command references
  std::fill(joint_position_references_.begin(), joint_position_references_.end(), 0.0);
  
  // Reset sequence numbers for topic mode
  if (use_topic_mode_) {
    ft_sequence_number_.store(0);
    last_processed_sequence_.store(0);
    consecutive_missed_updates_.store(0);
  }
  
  // Cache interface indices for fast access
  cacheInterfaceIndices();
  
  // Initialize joint positions from current state
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    if (i >= pos_state_indices_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Position state index out of bounds");
      return controller_interface::CallbackReturn::ERROR;
    }
    size_t idx = pos_state_indices_[i];
    if (idx >= state_interfaces_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "State interface index out of bounds for joint %s", 
        params_.joints[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    auto value = state_interfaces_[idx].get_optional();
    if (!value.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get position for joint %s", 
        params_.joints[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_positions_.at(i) = value.value();
    joint_position_references_.at(i) = joint_positions_.at(i);
  }
  
  // Ensure transform cache is ready
  updateTransforms();
  
  RCLCPP_INFO(get_node()->get_logger(), "AdmittanceController activated successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Starting deactivation cleanup...");
  
  // Reset services
  reset_pose_service_holder_.reset();
  move_to_pose_service_holder_.reset();
  
  // Reset subscriptions
  set_pose_sub_.reset();
  
  // Reset topic-based F/T sensor if used
  if (use_topic_mode_) {
    ft_topic_sub_.reset();
    ft_sequence_number_.store(0);
    last_processed_sequence_.store(0);
    consecutive_missed_updates_.store(0);
  }
  
  // Reset real-time publishers
  rt_cart_vel_pub_.reset();
  rt_pose_error_pub_.reset();
  
  // Clean up transform infrastructure
  RCLCPP_INFO(get_node()->get_logger(), "Cleaning up transform listeners...");
  tf_listener_.reset();
  tf_buffer_.reset();
  
  // Reset transform caches
  transform_base_ft_.reset();
  transform_base_tip_.reset();
  
  // Reset parameter buffer
  param_buffer_.writeFromNonRT(ur_admittance_controller::Params());
  
  // Reset kinematics
  kinematics_.reset();
  
  RCLCPP_INFO(get_node()->get_logger(), "Deactivation cleanup complete");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Cleaning up controller resources...");
  
  // Reset all shared pointers and release resources
  kinematics_.reset();
  kinematics_loader_.reset();
  
  // Reset realtime publishers
  rt_cart_vel_pub_.reset();
  rt_pose_error_pub_.reset();
  
  // Reset regular publishers
  desired_pose_pub_.reset();
  current_pose_pub_.reset();
  
  // Reset services
  reset_pose_service_holder_.reset();
  move_to_pose_service_holder_.reset();
  
  // Reset subscribers
  set_pose_sub_.reset();
  ft_topic_sub_.reset();
  
  // Reset TF resources
  tf_buffer_.reset();
  tf_listener_.reset();
  
  // Clear cached data
  pos_state_indices_.clear();
  ft_indices_.clear();
  
  // Reset transform caches
  transform_base_ft_.reset();
  transform_base_tip_.reset();
  
  RCLCPP_INFO(get_node()->get_logger(), "Controller cleanup complete");
  return controller_interface::CallbackReturn::SUCCESS;
}

void AdmittanceController::cacheInterfaceIndices()
{
  // Cache joint position state interface indices
  pos_state_indices_.resize(params_.joints.size());
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto name = params_.joints[i] + "/position";
    auto it = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
      [name](const auto & iface) { return iface.get_name() == name; });
    if (it != state_interfaces_.cend()) {
      pos_state_indices_[i] = std::distance(state_interfaces_.cbegin(), it);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "State interface '%s' not found!", name.c_str());
      pos_state_indices_[i] = std::numeric_limits<size_t>::max();  // Mark as invalid
    }
  }
  
  // Cache force/torque sensor interface indices (only in hardware mode)
  ft_indices_.resize(DOF, -1);
  if (!params_.ft_sensor_name.empty() && !use_topic_mode_) {
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

void AdmittanceController::ftSensorCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  // Use pre-allocated buffer (round-robin)
  static size_t buffer_idx = 0;
  auto& sensor_data = ft_buffer_storage_[buffer_idx];
  buffer_idx = (buffer_idx + 1) % ft_buffer_storage_.size();
  
  // Copy data without allocation
  sensor_data.msg.header = msg->header;
  sensor_data.msg.wrench = msg->wrench;
  sensor_data.receive_time = get_node()->get_clock()->now();
  
  // Cache transform if enabled
  if (params_.sensor_interface.topic_config.enable_transform_caching) {
    try {
      // Validate and determine source frame
      std::string source_frame = msg->header.frame_id;
      if (!params_.sensor_interface.topic_config.frame_id_override.empty()) {
        source_frame = params_.sensor_interface.topic_config.frame_id_override;
      }
      
      // Use actual frame from simulation if empty
      if (source_frame.empty()) {
        source_frame = "ft_sensor_link";  // Default for ur_simulation_gz
        RCLCPP_WARN_ONCE(get_node()->get_logger(), 
          "F/T sensor frame_id empty, using default: %s", source_frame.c_str());
      }
      
      // Check if transform is available first (non-blocking)
      if (tf_buffer_->canTransform(params_.base_link, source_frame, 
                                    tf2_ros::fromMsg(msg->header.stamp), 
                                    tf2::durationFromSec(0.0))) {
        // Lookup transform with zero timeout (non-blocking)
        geometry_msgs::msg::TransformStamped transform = 
          tf_buffer_->lookupTransform(
            params_.base_link,
            source_frame,
            tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0)
          );
        
        // Store as Eigen transform
        sensor_data.sensor_to_base_transform = tf2::transformToEigen(transform);
        sensor_data.transform_valid = true;
      } else {
        sensor_data.transform_valid = false;
      }
      
    } catch (const tf2::TransformException& ex) {
      // Use last valid transform if available
      sensor_data.transform_valid = false;
      // Don't log here - too frequent during startup
    }
  }
  
  // Atomic sequence increment
  ft_sequence_number_.fetch_add(1, std::memory_order_release);
  
  // Non-blocking write to RT buffer
  rt_ft_buffer_.writeFromNonRT(sensor_data);
}

bool AdmittanceController::handleFallbackStrategy()
{
  const std::string& strategy = params_.fallback_strategy.on_stale_data;
  
  if (strategy == "hold_position") {
    // Stop motion and hold current position
    V_base_tip_base_.setZero();
    desired_vel_.setZero();
    
    // Update joint references to current positions
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      size_t idx = pos_state_indices_[i];
      if (idx < state_interfaces_.size()) {
        auto value = state_interfaces_[idx].get_optional();
        if (value.has_value()) {
          joint_position_references_[i] = value.value();
        }
      }
    }
    return false;  // Skip control computation
    
  } else if (strategy == "use_last") {
    // Continue with last valid filtered wrench
    return checkDeadband();
    
  } else if (strategy == "zero_force") {
    // Assume no external forces
    wrench_filtered_.setZero();
    return checkDeadband();
  }
  
  return false;
}

bool AdmittanceController::validateSensorData(const FTSensorData* sensor_data, const rclcpp::Time& current_time)
{
  if (!sensor_data) {
    return false;
  }
  
  // Check data freshness
  double age_ms = (current_time - sensor_data->receive_time).seconds() * 1000.0;
  if (age_ms > params_.sensor_interface.topic_config.data_timeout_ms) {
    rtLogWarn(RTLogType::WARN_DEADBAND_ACTIVE, age_ms);
    return false;
  }
  
  return true;
}

}  // namespace ur_admittance_controller

