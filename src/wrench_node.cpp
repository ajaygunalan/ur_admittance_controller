// Wrench preprocessing node - applies gravity compensation and filtering to F/T sensor data
#include "wrench_node.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <filesystem>
#include <cmath>
#include <numbers>
#include <algorithm>

namespace ur_admittance_controller {

// Frame names as compile-time constants
static constexpr const char* BASE_FRAME = "base_link";
static constexpr const char* PAYLOAD_FRAME = "tool_payload";

// 6th order Butterworth coefficients for 10Hz @ 1000Hz sampling
static constexpr std::array<double, 7> BUTTERWORTH_B = {
    2.8982e-08, 1.7389e-07, 4.3473e-07, 5.7964e-07,
    4.3473e-07, 1.7389e-07, 2.8982e-08
};
static constexpr std::array<double, 7> BUTTERWORTH_A = {
    1.0000, -5.4914, 12.5825, -15.4051, 10.6453, -3.9318, 0.6054
};

WrenchNode::WrenchNode() : Node("wrench_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true))
{
    // Initialize 6th order Butterworth filter
    initializeFilter();
    
    // Load calibration
    const auto calib_file = declare_parameter<std::string>("calibration_file", 
        "config/wrench_calibration.yaml");
    const auto comp_type = declare_parameter<std::string>("compensation_type", "gravity_bias");
    
    // Validate calibration file
    if (!std::filesystem::exists(calib_file)) {
        RCLCPP_ERROR(get_logger(), "Calibration file not found: %s", calib_file.c_str());
        return;
    }
    
    // Create compensator
    compensator_ = createCompensator(comp_type, calib_file);
    RCLCPP_INFO(get_logger(), "Loaded %s compensator from: %s", 
        compensator_->getType().c_str(), calib_file.c_str());
    
    // Setup ROS2 interfaces
    using namespace std::placeholders;
    wrench_sub_ = create_subscription<WrenchMsg>(
        "/F_P_P_raw", rclcpp::SensorDataQoS(),
        std::bind(&WrenchNode::wrench_callback, this, _1));
    
    wrench_pub_ = create_publisher<WrenchMsg>("/F_P_B", rclcpp::SensorDataQoS());
}

void WrenchNode::wrench_callback(const WrenchMsg::ConstSharedPtr msg) {
    // Early exit if not ready
    if (!compensator_ || !tf_buffer_->canTransform(BASE_FRAME, PAYLOAD_FRAME, tf2::TimePointZero)) 
        return;
    
    // Get transform (needed for gravity compensation)
    const auto X_BP = tf2::transformToEigen(
        tf_buffer_->lookupTransform(BASE_FRAME, PAYLOAD_FRAME, tf2::TimePointZero));
    
    // Processing pipeline matching pulse_force_estimation EXACTLY:
    
    // 1. Extract raw data and apply Low-pass filter FIRST 
    const Wrench raw_wrench = extractWrench(*msg);
    const Wrench filtered = applyLowPassFilter(raw_wrench);
    
    // 2. Apply sensor bias correction  
    const Wrench bias_corrected = compensator_->applyBiasCorrection(filtered);
    
    // 3. Apply gravity compensation
    const Wrench compensated = compensator_->applyGravityCompensation(bias_corrected, X_BP);
    
    // 4. Frame transformation
    const Wrench transformed = transformWrench(compensated, X_BP.rotation());
    
    // 5. Threshold (deadband style like pulse)
    Wrench output;
    output[0] = std::abs(transformed[0]) <= FORCE_THRESHOLD_X ? 0.0 : transformed[0];
    output[1] = std::abs(transformed[1]) <= FORCE_THRESHOLD_Y ? 0.0 : transformed[1];
    output[2] = std::abs(transformed[2]) <= FORCE_THRESHOLD_Z ? 0.0 : transformed[2];
    output[3] = std::abs(transformed[3]) <= TORQUE_THRESHOLD_X ? 0.0 : transformed[3];
    output[4] = std::abs(transformed[4]) <= TORQUE_THRESHOLD_Y ? 0.0 : transformed[4];
    output[5] = std::abs(transformed[5]) <= TORQUE_THRESHOLD_Z ? 0.0 : transformed[5];
    
    // Publish if non-zero
    if (output.squaredNorm() > 0) {
        auto out_msg = std::make_unique<WrenchMsg>();
        out_msg->header = msg->header;
        out_msg->header.frame_id = BASE_FRAME;
        fillWrenchMsg(out_msg->wrench, output);
        wrench_pub_->publish(std::move(out_msg));
    }
}

void WrenchNode::initializeFilter() {
    // Initialize filter history buffers with zero wrenches
    std::fill(input_history_.begin(), input_history_.end(), Wrench::Zero());
    std::fill(output_history_.begin(), output_history_.end(), Wrench::Zero());
    
    RCLCPP_INFO(get_logger(), "Initialized 6th order Butterworth filter at %.1f Hz", 
        CUTOFF_FREQUENCY);
}

Wrench WrenchNode::applyLowPassFilter(const Wrench& input) {
    // Initialize filter history on first reading
    if (first_reading_) {
        first_reading_ = false;
        std::fill(input_history_.begin(), input_history_.end(), input);
        std::fill(output_history_.begin(), output_history_.end(), input);
        return input;
    }
    
    // Shift history buffers (newest at index 0)
    std::rotate(input_history_.rbegin(), input_history_.rbegin() + 1, input_history_.rend());
    std::rotate(output_history_.rbegin(), output_history_.rbegin() + 1, output_history_.rend());
    input_history_[0] = input;
    
    // Apply 6th order Butterworth difference equation:
    // y[n] = Σ(b[i]*x[n-i]) - Σ(a[i]*y[n-i]) for i=1..6
    Wrench output = BUTTERWORTH_B[0] * input_history_[0];
    
    for (size_t i = 1; i < 7; ++i) {
        output += BUTTERWORTH_B[i] * input_history_[i];   // Feedforward
        output -= BUTTERWORTH_A[i] * output_history_[i];   // Feedback
    }
    
    output_history_[0] = output;
    return output;
}

}  // namespace ur_admittance_controller

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ur_admittance_controller::WrenchNode>());
    rclcpp::shutdown();
    return 0;
}