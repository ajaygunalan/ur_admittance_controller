#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "wrench_compensation.hpp"
#include <ur_admittance_controller/utilities/error.hpp>
#include <ur_admittance_controller/utilities/types.hpp>
#include <memory>
#include <chrono>

namespace ur_admittance_controller {

class WrenchNode : public rclcpp::Node {
public:
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;
    
    WrenchNode();
    
private:
    void wrench_callback(const WrenchMsg::ConstSharedPtr msg);
    Status loadCalibrationParams();
    void setupROSInterfaces();
    void computeSensorToProbeAdjoint();
    
    
    // Yu pipeline state
    Wrench f_raw_s_;
    Transform X_TB;
    Vector3d f_grav_s_;
    Wrench ft_proc_s_;  // Compensated in sensor frame
    Wrench wrench_probe;  // Compensated wrench at probe location
    Wrench ft_proc_b_;  // Transformed to base frame
    
    // Adjoint transformation
    Eigen::Matrix<double, 6, 6> adjoint_probe_sensor;
    
    // Calibration parameters loaded once from YAML
    Matrix3d R_SE_;        // Rotation sensor to end-effector
    Vector3d f_grav_b_;    // Gravity vector in base frame
    Vector3d f_bias_s_;    // Force bias in sensor frame
    Vector3d t_bias_s_;    // Torque bias in sensor frame
    Vector3d p_CoM_s_;     // Center of mass in sensor frame
    
    // Consolidated calibration params for algorithm
    GravityCompensationParams calibration_params_;
    
    // TF2 for robot kinematics
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // ROS2 interfaces
    rclcpp::Subscription<WrenchMsg>::SharedPtr wrench_sub_;
    rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_proc_sensor_pub_;
    rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_proc_probe_pub_;
    rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_proc_probe_base_pub_;
};

} // namespace ur_admittance_controller