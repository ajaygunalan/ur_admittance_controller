#pragma once

#include <string>
#include <Eigen/Core>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/time.hpp>

#include <utilities/types.hpp>

namespace ur_admittance_controller {
namespace conversions {

// Convert ROS WrenchStamped to Eigen 6D wrench
inline Wrench6d FromMsg(const geometry_msgs::msg::WrenchStamped& msg) {
    return (Wrench6d() << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                          msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z).finished();
}

// Convert ROS Wrench to Eigen 6D wrench
inline Wrench6d FromMsg(const geometry_msgs::msg::Wrench& wrench) {
    return (Wrench6d() << wrench.force.x, wrench.force.y, wrench.force.z,
                          wrench.torque.x, wrench.torque.y, wrench.torque.z).finished();
}

// Fill WrenchStamped message with Eigen wrench
inline void FillMsg(const Wrench6d& wrench, geometry_msgs::msg::WrenchStamped& msg) {
    msg.wrench.force.x = wrench[0];
    msg.wrench.force.y = wrench[1];
    msg.wrench.force.z = wrench[2];
    msg.wrench.torque.x = wrench[3];
    msg.wrench.torque.y = wrench[4];
    msg.wrench.torque.z = wrench[5];
}

// Convert Eigen 6D wrench to ROS WrenchStamped (implemented in .cpp)
geometry_msgs::msg::WrenchStamped ToMsg(
    const Wrench6d& wrench,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

} // namespace conversions
} // namespace ur_admittance_controller