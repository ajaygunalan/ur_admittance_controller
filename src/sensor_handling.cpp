// Force/torque sensor and transform management implementation

#include "admittance_node.hpp"

namespace ur_admittance_controller {




Vector6d AdmittanceNode::transformWrench(const Vector6d& wrench_sensor_frame)
{
  if (params_.ft_frame == params_.base_link) {
    return wrench_sensor_frame;  // No transform needed
  }
  
  try {
    auto transform = tf_buffer_->lookupTransform(
      params_.base_link, 
      params_.ft_frame,
      tf2::TimePointZero,  // Latest available
      std::chrono::milliseconds(50));  // 50ms timeout
    
    // Compute adjoint matrix for wrench transformation
    Eigen::Isometry3d T = tf2::transformToEigen(transform);
    Eigen::Matrix3d R = T.rotation();
    Eigen::Vector3d t = T.translation();
    
    Matrix6d adjoint = Matrix6d::Zero();
    adjoint.block<3, 3>(0, 0) = R;
    adjoint.block<3, 3>(3, 3) = R;
    
    Eigen::Matrix3d t_cross;
    t_cross << 0, -t.z(), t.y(),
               t.z(), 0, -t.x(),
               -t.y(), t.x(), 0;
    adjoint.block<3, 3>(3, 0) = -t_cross * R;  // FIXED: Correct sign for wrench transformation
    
    return adjoint * wrench_sensor_frame;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "F/T transform lookup failed: %s, using sensor frame data", ex.what());
    return wrench_sensor_frame;  // Fallback
  }
}

bool AdmittanceNode::getCurrentEndEffectorPose(Eigen::Isometry3d& pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      params_.base_link,
      params_.tip_link, 
      tf2::TimePointZero,
      std::chrono::milliseconds(50));
    
    pose = tf2::transformToEigen(transform);
    return true;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "End-effector transform lookup failed: %s", ex.what());
    return false;
  }
}


// Check if forces exceed deadband threshold
bool AdmittanceNode::checkDeadband()
{
  // Check if any force/torque exceeds threshold
  for (size_t i = 0; i < 6; ++i) {
    if (std::abs(wrench_filtered_(i)) > params_.admittance.min_motion_threshold) {
      return true;
    }
  }
  
  // All forces below threshold - stop motion
  V_base_tip_base_.setZero();
  
  return false;
}


}  // namespace ur_admittance_controller
