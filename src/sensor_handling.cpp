// Force/torque sensor processing and coordinate frame transformation utilities
// Handles F/T sensor data filtering and coordinate transforms using TF2

#include "admittance_node.hpp"

namespace ur_admittance_controller {

// Transform 6-DOF wrench from sensor frame to base frame using TF2
Vector6d AdmittanceNode::TransformWrench(const Vector6d& wrench_sensor_frame) {
  if (params_.ft_frame == params_.base_link) {
    return wrench_sensor_frame;  // Already in base frame
  }
  try {
    // Look up transform from sensor to base frame
    const auto transform = tf_buffer_->lookupTransform(
        params_.base_link,
        params_.ft_frame,
        tf2::TimePointZero,                    // Latest available
        std::chrono::milliseconds(50));        // 50ms timeout
    // Convert TF2 transform to Eigen representation
    const Eigen::Isometry3d X_base_ft = tf2::transformToEigen(transform);
    const Eigen::Matrix3d R_base_ft = X_base_ft.rotation();
    const Eigen::Vector3d p_base_ft = X_base_ft.translation();
    // Build 6x6 adjoint matrix for wrench transformation
    Matrix6d adjoint_base_ft = Matrix6d::Zero();
    adjoint_base_ft.block<3, 3>(0, 0) = R_base_ft;
    adjoint_base_ft.block<3, 3>(3, 3) = R_base_ft;

    // Skew-symmetric matrix for cross product: [p]×
    Eigen::Matrix3d p_cross;
    p_cross << 0, -p_base_ft.z(), p_base_ft.y(),
               p_base_ft.z(), 0, -p_base_ft.x(),
               -p_base_ft.y(), p_base_ft.x(), 0;
    adjoint_base_ft.block<3, 3>(3, 0) = -p_cross * R_base_ft;

    return adjoint_base_ft * wrench_sensor_frame;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Transform lookup failed (ft->base): %s, using raw sensor data", ex.what());
    return wrench_sensor_frame;  // Fallback to untransformed data
  }
}

// Get current end-effector pose in base frame using TF2 transforms
bool AdmittanceNode::GetCurrentEndEffectorPose(Eigen::Isometry3d& pose) {
  try {
    // Look up transform from base to end-effector
    const auto transform = tf_buffer_->lookupTransform(
        params_.base_link,
        params_.tip_link,
        tf2::TimePointZero,
        std::chrono::milliseconds(50));
    // Convert to Eigen pose representation
    pose = tf2::transformToEigen(transform);
    return true;

  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Transform lookup failed (base->tip): %s", ex.what());
    return false;
  }
}

// Check if filtered forces/torques exceed motion threshold (deadband)
bool AdmittanceNode::CheckDeadband() {
  // Check if any force/torque component exceeds minimum threshold
  for (size_t i = 0; i < 6; ++i) {
    if (std::abs(F_sensor_filtered_(i)) > params_.admittance.min_motion_threshold) {
      return true;  // Motion should be allowed
    }
  }

  // All forces below threshold - robot should remain stationary
  return false;
}


}  // namespace ur_admittance_controller
