#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "wrench_compensation.hpp"
#include <memory>
#include <chrono>

namespace ur_admittance_controller {

class WrenchNode : public rclcpp::Node {
public:
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;
    
    WrenchNode();
    
private:
    void wrench_callback(const WrenchMsg::ConstSharedPtr msg);
    
    
    // Yu pipeline state
    Wrench f_raw_s_;
    Transform X_EB_;
    Vector3d f_grav_s_;
    Wrench ft_proc_s_;  // Compensated in payload frame
    Wrench ft_proc_b_;  // Transformed to base frame
    
    // Yu et al. bias and gravity compensator
    std::unique_ptr<WrenchCompensator> compensator_;
    
    // TF2 for robot kinematics
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // ROS2 interfaces
    rclcpp::Subscription<WrenchMsg>::SharedPtr wrench_sub_;
    rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_sensor_pub_;
};

} // namespace ur_admittance_controller