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
    
    WrenchNode();  // May throw on initialization failure
    
private:
    void wrench_callback(const WrenchMsg::ConstSharedPtr msg);
    void initializeFilter();
    Wrench applyLowPassFilter(const Wrench& input);
    
    // Filtering and compensation
    std::unique_ptr<WrenchCompensator> compensator_;
    bool first_reading_{true};
    
    // 6th order Butterworth filter state
    std::array<Wrench, 7> input_history_;  // Previous inputs
    std::array<Wrench, 7> output_history_; // Previous outputs
    
    
    // TF2 for coordinate transforms
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // ROS2 interfaces
    rclcpp::Subscription<WrenchMsg>::SharedPtr wrench_sub_;
    rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_pub_;
    
    // Filter and threshold constants
    static constexpr double CUTOFF_FREQUENCY = 10.0;   // Hz
    static constexpr double SAMPLING_FREQUENCY = 1000.0; // Hz (typical F/T sensor rate)
    static constexpr double FORCE_THRESHOLD_X = 1.5;   // N
    static constexpr double FORCE_THRESHOLD_Y = 1.5;   // N
    static constexpr double FORCE_THRESHOLD_Z = 1.5;   // N
    static constexpr double TORQUE_THRESHOLD_X = 1.5;  // Nm
    static constexpr double TORQUE_THRESHOLD_Y = 1.5;  // Nm
    static constexpr double TORQUE_THRESHOLD_Z = 1.5;  // Nm
};

} // namespace ur_admittance_controller