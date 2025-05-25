/**
 * @file communication_interface.cpp
 * @brief External communication handlers for UR Admittance Controller
 * 
 * This file contains ROS service handlers and subscription callbacks
 * that interface with external systems. These are non-real-time operations.
 */

#include "admittance_controller.hpp"
#include <memory>
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace ur_admittance_controller {

// Service callback implementations for impedance mode control
void AdmittanceController::handle_reset_pose(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    // Set desired pose to current pose (reset error to zero)
    if (ee_transform_cache_.isValid()) {
      desired_pose_ = current_pose_;
      
      // Publish the current/desired pose for visualization
      auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
      pose_msg->header.stamp = get_node()->now();
      pose_msg->header.frame_id = params_.base_link;
      
      // Convert from Eigen to ROS message
      Eigen::Quaterniond q(current_pose_.linear());
      pose_msg->pose.orientation.w = q.w();
      pose_msg->pose.orientation.x = q.x();
      pose_msg->pose.orientation.y = q.y();
      pose_msg->pose.orientation.z = q.z();
      
      pose_msg->pose.position.x = current_pose_.translation().x();
      pose_msg->pose.position.y = current_pose_.translation().y();
      pose_msg->pose.position.z = current_pose_.translation().z();
      
      // Publish to both topics since they're now identical
      current_pose_pub_->publish(*pose_msg);
      desired_pose_pub_->publish(*pose_msg);
      
      response->success = true;
      response->message = "Desired pose reset to current pose successfully";
      RCLCPP_INFO(get_node()->get_logger(), "Desired pose reset to current pose");
    } else {
      response->success = false;
      response->message = "Failed to reset pose: No valid current pose available";
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset pose: No valid current pose");
    }
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string("Exception during pose reset: ") + e.what();
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during pose reset: %s", e.what());
  }
}

void AdmittanceController::handle_set_desired_pose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  try {
    // Transform the requested pose to the base frame if needed
    if (msg->header.frame_id != params_.base_link && !msg->header.frame_id.empty()) {
      RCLCPP_INFO(get_node()->get_logger(),
        "Transforming desired pose from %s to %s",
        msg->header.frame_id.c_str(), params_.base_link.c_str());
      
      // Look up the transform
      auto transform = tf_buffer_->lookupTransform(
        params_.base_link, msg->header.frame_id,
        tf2::timeFromSec(0), tf2::durationFromSec(1.0));
      
      // Transform the pose
      geometry_msgs::msg::PoseStamped pose_out;
      tf2::doTransform(*msg, pose_out, transform);
      
      // Set the desired pose from the transformed pose
      Eigen::Quaterniond q(pose_out.pose.orientation.w, pose_out.pose.orientation.x,
                         pose_out.pose.orientation.y, pose_out.pose.orientation.z);
                         
      Eigen::Vector3d p(pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);
      
      desired_pose_.linear() = q.toRotationMatrix();
      desired_pose_.translation() = p;
    } else {
      // Directly use the pose since it's already in the base frame
      Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
                         
      Eigen::Vector3d p(msg->pose.position.x, msg->pose.position.y,
                      msg->pose.position.z);
      
      desired_pose_.linear() = q.toRotationMatrix();
      desired_pose_.translation() = p;
    }
    
    // Publish the updated desired pose for visualization
    auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    pose_msg->header.stamp = get_node()->now();
    pose_msg->header.frame_id = params_.base_link;
    
    // Convert from Eigen to ROS message
    Eigen::Quaterniond q(desired_pose_.linear());
    pose_msg->pose.orientation.w = q.w();
    pose_msg->pose.orientation.x = q.x();
    pose_msg->pose.orientation.y = q.y();
    pose_msg->pose.orientation.z = q.z();
    
    pose_msg->pose.position.x = desired_pose_.translation().x();
    pose_msg->pose.position.y = desired_pose_.translation().y();
    pose_msg->pose.position.z = desired_pose_.translation().z();
    
    desired_pose_pub_->publish(*pose_msg);
    
    RCLCPP_INFO(get_node()->get_logger(), "Desired pose updated successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during set_desired_pose: %s", e.what());
  }
}

void AdmittanceController::handle_move_to_pose(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    // Safety check: Don't allow move if already executing trajectory
    if (executing_trajectory_) {
      response->success = false;
      response->message = "Already executing trajectory. Rejecting move request.";
      RCLCPP_WARN(get_node()->get_logger(), response->message.c_str());
      return;
    }
    
    // Store the current pose as the target - we'll just use our current pose as target
    // In a more complete implementation, we might use a predefined home or target position
    auto current_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    current_pose_msg->header.stamp = get_node()->now();
    current_pose_msg->header.frame_id = params_.base_link;
    
    // Convert Eigen pose to geometry_msgs pose
    current_pose_msg->pose.position.x = current_pose_.translation().x();
    current_pose_msg->pose.position.y = current_pose_.translation().y();
    current_pose_msg->pose.position.z = current_pose_.translation().z();
    
    Eigen::Quaterniond q(current_pose_.rotation());
    current_pose_msg->pose.orientation.x = q.x();
    current_pose_msg->pose.orientation.y = q.y();
    current_pose_msg->pose.orientation.z = q.z();
    current_pose_msg->pose.orientation.w = q.w();
    
    // Store as pending desired pose
    pending_desired_pose_ = *current_pose_msg;
    
    // Start executing trajectory using reference interfaces
    executing_trajectory_ = true;
    
    // Set the target as the current reference pose
    // The controller will automatically transition to this pose through
    // the update_reference_interfaces() method in the control loop
    RCLCPP_INFO(get_node()->get_logger(), "Setting target pose through reference interfaces");
    
    // Signal success
    response->success = true;
    response->message = "Target pose set successfully";
    
    // Setup callback group for the timer
    auto callback_group = get_node()->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
      
    // In a complete implementation, add callback for trajectory completion
    // For now, set a timer to simulate trajectory completion
    auto timer = get_node()->create_wall_timer(
      std::chrono::duration<double>(safe_startup_params_.trajectory_duration),
      [this]() {
        // Trajectory completed
        executing_trajectory_ = false;
        
        // Use stiffness_recently_changed_ to trigger gradual engagement
        stiffness_recently_changed_ = true;
        stiffness_engagement_factor_ = 0.0;
        
        RCLCPP_INFO(get_node()->get_logger(), 
          "Safe movement completed, starting gradual stiffness engagement");
      },
      callback_group
    );
    
    response->success = true;
    response->message = "Moving to target position before enabling impedance control";
    RCLCPP_INFO(get_node()->get_logger(), response->message.c_str());
  } catch (const std::exception & e) {
    executing_trajectory_ = false;
    response->success = false;
    response->message = std::string("Exception during move_to_pose: ") + e.what();
    RCLCPP_ERROR(get_node()->get_logger(), response->message.c_str());
  }
}

} // namespace ur_admittance_controller
