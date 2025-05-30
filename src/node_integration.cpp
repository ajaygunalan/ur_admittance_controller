
#include "admittance_node.hpp"

namespace ur_admittance_controller {

bool AdmittanceNode::waitForTransforms()
{
  // Check transforms without timeout to avoid timer-related segfaults
  // Following Nav2 pattern: use retry loop in on_activate() instead
  std::string error;
  
  // Check base transforms
  bool base_transforms_available = 
    tf_buffer_->canTransform(params_.world_frame, params_.base_link, 
                            tf2::TimePointZero, &error);
  
  if (!base_transforms_available) {
    RCLCPP_WARN(get_logger(), 
      "Transform from %s to %s not yet available: %s", 
      params_.world_frame.c_str(), params_.base_link.c_str(), error.c_str());
    return false;
  }
  
  // Check tip transform
  error.clear();
  bool tip_transform_available = 
    tf_buffer_->canTransform(params_.base_link, params_.tip_link, 
                            tf2::TimePointZero, &error);
  
  if (!tip_transform_available) {
    RCLCPP_WARN(get_logger(), 
      "Transform from %s to %s not yet available: %s", 
      params_.base_link.c_str(), params_.tip_link.c_str(), error.c_str());
    return false;
  }
  
  // Check F/T sensor transform (may differ in simulation vs hardware)
  error.clear();
  bool ft_transform_available = 
    (params_.ft_frame == params_.base_link) || 
    tf_buffer_->canTransform(params_.base_link, params_.ft_frame, 
                            tf2::TimePointZero, &error);
  
  if (!ft_transform_available) {
    RCLCPP_WARN(get_logger(), 
      "Transform from %s to %s not yet available: %s", 
      params_.base_link.c_str(), params_.ft_frame.c_str(), error.c_str());
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "All required transforms are available");
  
  transform_base_ft_.reset();
  transform_base_tip_.reset();
  
  transform_base_ft_.target_frame = params_.base_link;
  transform_base_ft_.source_frame = params_.ft_frame;
  
  transform_base_tip_.target_frame = params_.base_link;
  transform_base_tip_.source_frame = params_.tip_link;
  
  transform_update_needed_.store(true);
  
  try {
    updateTransformCaches();
    
    bool ee_valid = transform_base_tip_.isValid();
    bool ft_valid = (params_.ft_frame == params_.base_link) || transform_base_ft_.isValid();
    
    if (!ee_valid || !ft_valid) {
      RCLCPP_ERROR(get_logger(), 
        "Failed to initialize transform caches - EE: %s, F/T: %s", 
        ee_valid ? "OK" : "FAILED", 
        ft_valid ? "OK" : "FAILED");
      return false;
    }
    
    return true;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "Exception during transform cache initialization: %s", ex.what());
    return false;
  }
}

// loadKinematics() is defined in admittance_node.cpp

}

