// F/T sensor bias and gravity compensation node (Yu et al. method)
#include "wrench_node.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <filesystem>

namespace ur_admittance_controller {

// Frame names for ur_admittance_controller kinematic tree
static constexpr const char* BASE_FRAME = "base_link";
static constexpr const char* EE_FRAME = "tool_payload";
static constexpr const char* SENSOR_FRAME = "tool_payload";

WrenchNode::WrenchNode() : Node("wrench_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true))
{
    // Load calibration parameters
    const auto calib_file = declare_parameter<std::string>("calibration_file", 
        std::string(PACKAGE_SOURCE_DIR) + "/config/wrench_calibration.yaml");
    const auto comp_type = declare_parameter<std::string>("compensation_type", "gravity_bias");
    
    if (!std::filesystem::exists(calib_file)) {
        RCLCPP_ERROR(get_logger(), "Calibration file not found: %s", calib_file.c_str());
        return;
    }
    
    // Create compensator following Yu et al. method
    compensator_ = createCompensator(comp_type, calib_file);
    RCLCPP_INFO(get_logger(), "Yu bias compensation loaded from: %s", calib_file.c_str());
    
    // Setup ROS2 interfaces (ur_admittance_controller topics)
    using namespace std::placeholders;
    wrench_sub_ = create_subscription<WrenchMsg>(
        "/F_P_P_raw", rclcpp::SensorDataQoS(),
        std::bind(&WrenchNode::wrench_callback, this, _1));
    
    // Output for admittance controller
    wrench_sensor_pub_ = create_publisher<WrenchMsg>("/F_P_B", rclcpp::SensorDataQoS());
    
    // Initialize pipeline state
    f_raw_s_ = Wrench::Zero();
    X_EB_ = Transform::Identity();
    f_grav_s_ = Vector3d::Zero();
    ft_proc_s_ = Wrench::Zero();
    ft_proc_b_ = Wrench::Zero();
}

void WrenchNode::wrench_callback(const WrenchMsg::ConstSharedPtr msg) {
    auto* gravity_comp = static_cast<GravityCompensator*>(compensator_.get());
    if (!gravity_comp || !tf_buffer_->canTransform(EE_FRAME, BASE_FRAME, tf2::TimePointZero)) {
        return;
    }
    
    // STEP 1: Extract raw F/T data from sensor
    f_raw_s_ = extractWrench(*msg);
    
    // STEP 2: Get robot end-effector to base transform (X_EB)
    X_EB_ = tf2::transformToEigen(tf_buffer_->lookupTransform(EE_FRAME, BASE_FRAME, tf2::TimePointZero));
    
    // STEP 3: Extract rotation matrix R_EB from transform
    const Matrix3d R_EB = X_EB_.rotation();
    
    // STEP 4: Compute gravity force in sensor frame (Yu et al. Equation 10)
    // f_grav_s = R_SE × R_EB × f_grav_b
    f_grav_s_ = gravity_comp->get_R_SE() * R_EB * gravity_comp->get_f_grav_b();
    
    // STEP 5: Force compensation (Yu et al. Equation 10)
    // F_compensated = F_raw - F_gravity - F_bias
    ft_proc_s_.head<3>() = f_raw_s_.head<3>() - f_grav_s_ - gravity_comp->get_f_bias_s();
    
    // STEP 6: Torque compensation (Yu et al. Equation 11)
    // T_compensated = T_raw - (p_CoM × F_gravity) - T_bias
    const Vector3d gravity_torque = gravity_comp->skew_symmetric(gravity_comp->get_p_CoM_s()) * f_grav_s_;
    ft_proc_s_.tail<3>() = f_raw_s_.tail<3>() - gravity_torque - gravity_comp->get_t_bias_s();
    
    // STEP 7: Transform compensated wrench from payload to base frame
    // TODO: Implement proper R_BP rotation - for now using identity
    ft_proc_b_ = ft_proc_s_;
    
    // STEP 8: Publish compensated wrench in base frame for admittance controller
    WrenchMsg proc_msg;
    proc_msg.header = msg->header;
    proc_msg.header.frame_id = BASE_FRAME;
    fillWrenchMsg(proc_msg.wrench, ft_proc_b_);
    wrench_sensor_pub_->publish(proc_msg);
}



}  // namespace ur_admittance_controller

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ur_admittance_controller::WrenchNode>();
    RCLCPP_INFO(node->get_logger(), "Yu bias compensation node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}