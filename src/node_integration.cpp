
#include "admittance_node.hpp"

namespace ur_admittance_controller {

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

// loadKinematics() is defined in admittance_node.cpp

}

