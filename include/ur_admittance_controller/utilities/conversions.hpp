#pragma once

#include <ur_admittance_controller/utilities/types.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/time.hpp>

namespace ur_admittance_controller::conversions {

// =============================================================================
// Wrench Conversions
// =============================================================================

/**
 * Convert ROS WrenchStamped message to Eigen 6D wrench vector
 * @param msg ROS wrench message
 * @return 6D wrench vector [fx, fy, fz, tx, ty, tz]
 */
inline Wrench6d fromMsg(const geometry_msgs::msg::WrenchStamped& msg) {
    Wrench6d wrench;
    wrench << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
              msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
    return wrench;
}

/**
 * Convert ROS Wrench message to Eigen 6D wrench vector
 * @param wrench ROS wrench message (not stamped)
 * @return 6D wrench vector [fx, fy, fz, tx, ty, tz]
 */
inline Wrench6d fromMsg(const geometry_msgs::msg::Wrench& wrench) {
    Wrench6d result;
    result << wrench.force.x, wrench.force.y, wrench.force.z,
              wrench.torque.x, wrench.torque.y, wrench.torque.z;
    return result;
}

/**
 * Convert Eigen 6D wrench vector to ROS WrenchStamped message
 * @param wrench 6D wrench vector [fx, fy, fz, tx, ty, tz]
 * @param frame_id Reference frame for the wrench
 * @param stamp Time stamp for the message
 * @return ROS WrenchStamped message
 */
inline geometry_msgs::msg::WrenchStamped toMsg(
    const Wrench6d& wrench, 
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.wrench.force.x = wrench[0];
    msg.wrench.force.y = wrench[1];
    msg.wrench.force.z = wrench[2];
    msg.wrench.torque.x = wrench[3];
    msg.wrench.torque.y = wrench[4];
    msg.wrench.torque.z = wrench[5];
    return msg;
}

/**
 * Fill an existing WrenchStamped message with Eigen wrench data
 * @param wrench 6D wrench vector to convert
 * @param msg Message to fill (header should already be set)
 */
inline void fillMsg(const Wrench6d& wrench, geometry_msgs::msg::WrenchStamped& msg) {
    msg.wrench.force.x = wrench[0];
    msg.wrench.force.y = wrench[1];
    msg.wrench.force.z = wrench[2];
    msg.wrench.torque.x = wrench[3];
    msg.wrench.torque.y = wrench[4];
    msg.wrench.torque.z = wrench[5];
}

// =============================================================================
// Joint State Conversions
// =============================================================================

/**
 * Extract joint positions from JointState message in correct order
 * @param msg Joint state message
 * @param joint_names Expected joint names in order
 * @return Joint positions vector
 */
inline JointVector extractJointPositions(
    const sensor_msgs::msg::JointState& msg,
    const JointNames& joint_names) {
    
    JointVector positions(joint_names.size());
    
    for (size_t i = 0; i < joint_names.size(); ++i) {
        auto it = std::find(msg.name.begin(), msg.name.end(), joint_names[i]);
        if (it != msg.name.end()) {
            size_t idx = std::distance(msg.name.begin(), it);
            positions[i] = msg.position[idx];
        }
    }
    
    return positions;
}

/**
 * Extract joint velocities from JointState message in correct order
 * @param msg Joint state message
 * @param joint_names Expected joint names in order
 * @return Joint velocities vector
 */
inline JointVector extractJointVelocities(
    const sensor_msgs::msg::JointState& msg,
    const JointNames& joint_names) {
    
    JointVector velocities(joint_names.size());
    
    for (size_t i = 0; i < joint_names.size(); ++i) {
        auto it = std::find(msg.name.begin(), msg.name.end(), joint_names[i]);
        if (it != msg.name.end()) {
            size_t idx = std::distance(msg.name.begin(), it);
            velocities[i] = msg.velocity[idx];
        }
    }
    
    return velocities;
}

} // namespace ur_admittance_controller::conversions