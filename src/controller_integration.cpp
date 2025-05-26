/**
 * @file controller_integration.cpp
 * @brief External system integration for UR Admittance Controller
 * 
 * Handles integration with TF2 transforms, kinematics plugins,
 * URDF parsing, and hardware interface validation.
 */

#include "admittance_controller.hpp"

namespace ur_admittance_controller {

bool AdmittanceController::waitForTransforms()
{
  const auto timeout = rclcpp::Duration::from_seconds(5.0);
  std::string error;
  auto now = get_node()->get_clock()->now();
  
  // Always check world->base and base->tip transforms
  bool transforms_available = 
    tf_buffer_->canTransform(params_.world_frame, params_.base_link, 
                            rclcpp::Time(0), timeout, &error) &&
    tf_buffer_->canTransform(params_.base_link, params_.tip_link, 
                            rclcpp::Time(0), timeout, &error);
  
  // Only check F/T transform if sensor frame differs from control frame
  if (params_.ft_frame != params_.base_link) {
    transforms_available = transforms_available && 
      tf_buffer_->canTransform(params_.base_link, params_.ft_frame, 
                              rclcpp::Time(0), timeout, &error);
  }
  
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
    // For Scenario B (ft_frame == base_link), F/T transform cache might not be valid
    bool ee_valid = ee_transform_cache_.isValid();
    bool ft_valid = (params_.ft_frame == params_.base_link) || ft_transform_cache_.isValid();
    
    if (!ee_valid || !ft_valid) {
      RCLCPP_ERROR(get_node()->get_logger(), 
        "Failed to initialize transform caches - EE: %s, F/T: %s", 
        ee_valid ? "OK" : "FAILED", 
        ft_valid ? "OK" : "FAILED");
      return false;
    }
    
    return true;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during transform cache initialization: %s", ex.what());
    return false;
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
