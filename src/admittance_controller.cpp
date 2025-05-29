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
  
  // Configure command interfaces for downstream controller
  if (!params_.downstream_controller_name.empty()) {
    for (const auto & joint : params_.joints) {
      config.names.push_back(
        params_.downstream_controller_name + "/" + joint + "/position");
    }
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
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    reference_interfaces.emplace_back(
      params_.joints[i], "position", &joint_position_references_[i]);
  }
  
  return reference_interfaces;
}

controller_interface::CallbackReturn AdmittanceController::on_init()
{
  try {
    // Initialize parameter listener
    param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
    
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
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!waitForTransforms()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get transforms");
    return controller_interface::CallbackReturn::ERROR;
  }
  
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
  
  cacheInterfaceIndices();
  
  if (!params_.downstream_controller_name.empty() && !command_interfaces_.empty()) {
    cmd_interface_to_joint_index_.clear();
    cmd_interface_to_joint_index_.resize(command_interfaces_.size(), 0);
    
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      const std::string& interface_name = command_interfaces_[i].get_interface_name();
      const std::string& prefix_name = command_interfaces_[i].get_prefix_name();
      
      std::string joint_name;
      size_t last_slash = prefix_name.find_last_of('/');
      if (last_slash != std::string::npos) {
        joint_name = prefix_name.substr(last_slash + 1);
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), 
          "Invalid command interface format: %s", prefix_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      
      auto it = std::find(params_.joints.begin(), params_.joints.end(), joint_name);
      if (it != params_.joints.end()) {
        size_t joint_idx = std::distance(params_.joints.begin(), it);
        cmd_interface_to_joint_index_[i] = joint_idx;
        RCLCPP_DEBUG(get_node()->get_logger(), 
          "Mapped command interface %s to joint index %zu", 
          (prefix_name + "/" + interface_name).c_str(), joint_idx);
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), 
          "Joint %s not found in controller joints", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }
  
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    if (i >= pos_state_indices_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Position state index out of bounds");
      return controller_interface::CallbackReturn::ERROR;
    }
    size_t idx = pos_state_indices_[i];
    if (idx >= state_interfaces_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "State interface index out of bounds");
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_positions_.at(i) = state_interfaces_[idx].get_optional().value();
    joint_position_references_.at(i) = joint_positions_.at(i);
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "AdmittanceController activated");
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

void AdmittanceController::cacheInterfaceIndices()
{
  // Cache joint position state interface indices
  pos_state_indices_.resize(params_.joints.size());
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto name = params_.joints[i] + "/position";
    auto it = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
      [name](const auto & iface) { return iface.get_name() == name; });
    pos_state_indices_[i] = std::distance(state_interfaces_.cbegin(), it);
  }
  
  // Cache force/torque sensor interface indices
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


}  // namespace ur_admittance_controller

