// F/T sensor bias and gravity compensation node (Yu et al. method)
#include "wrench_node.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace ur_admittance_controller {

// Frame names for ur_admittance_controller kinematic tree
static constexpr const char* BASE_FRAME = "base_link";
static constexpr const char* EE_FRAME = "tool_payload";
static constexpr const char* SENSOR_FRAME = "tool_payload";

WrenchNode::WrenchNode() : Node("wrench_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true))  // Enable tf2 caching for real-time lookup
{
    // CRITICAL: Load calibration first - early exit prevents invalid operation
    if (!loadCalibrationParams()) return;
    
    // DESIGN: SensorDataQoS ensures reliable, real-time F/T data flow (~100-1000Hz)
    wrench_sub_ = create_subscription<WrenchMsg>("/netft/raw_sensor", rclcpp::SensorDataQoS(),
        std::bind(&WrenchNode::wrench_callback, this, std::placeholders::_1));
    wrench_proc_sensor_pub_ = create_publisher<WrenchMsg>("/netft/proc_sensor", rclcpp::SensorDataQoS());
    wrench_proc_probe_pub_ = create_publisher<WrenchMsg>("/netft/proc_probe", rclcpp::SensorDataQoS());
    wrench_proc_probe_base_pub_ = create_publisher<WrenchMsg>("/netft/proc_probe_base", rclcpp::SensorDataQoS());
    
    // PERFORMANCE: Separate by type to avoid Eigen size mismatch in chained assignment
    f_raw_s_ = ft_proc_s_ = ft_proc_b_ = Wrench::Zero();  // 6D wrenches (force+torque)
    f_grav_s_ = Vector3d::Zero();                         // 3D gravity force only
    X_EB_ = Transform::Identity();                        // Robot pose transform
}

void WrenchNode::wrench_callback(const WrenchMsg::ConstSharedPtr msg) {
    if (!tf_buffer_->canTransform(EE_FRAME, BASE_FRAME, tf2::TimePointZero)) {
        return;
    }
    
    // STEP 1: Extract raw F/T data from sensor (ROS WrenchStamped → Eigen 6D vector)
    f_raw_s_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,    // Forces [N]
                msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;  // Torques [Nm]
    
    // STEP 2: Get robot end-effector to base transform (X_EB)
    X_EB_ = tf2::transformToEigen(tf_buffer_->lookupTransform(EE_FRAME, BASE_FRAME, tf2::TimePointZero));
    
    // STEP 3: Extract rotation matrix R_EB from transform
    const Matrix3d R_EB = X_EB_.rotation();
    
    // STEP 4: Compute gravity force in sensor frame (Yu et al. Equation 10)
    // f_grav_s = R_SE × R_EB × f_grav_b
    f_grav_s_ = R_SE_ * R_EB * f_grav_b_;
    
    // STEP 5: Force compensation (Yu et al. Equation 10)
    // F_compensated = F_raw - F_gravity - F_bias
    ft_proc_s_.head<3>() = f_raw_s_.head<3>() - f_grav_s_ - f_bias_s_;
    
    // STEP 6: Torque compensation (Yu et al. Equation 11)
    // T_compensated = T_raw - (p_CoM × F_gravity) - T_bias
    const Vector3d gravity_torque = p_CoM_s_.cross(f_grav_s_);
    ft_proc_s_.tail<3>() = f_raw_s_.tail<3>() - gravity_torque - t_bias_s_;
    
    // STEP 7: Transform compensated wrench from probe to base frame using adjoint matrix
    // Apply adjoint transformation: w_base = Ad(T_base_probe) * w_probe
    const Matrix3d R_BP = X_EB_.rotation().transpose();  // Base to Probe rotation
    const Vector3d p_BP = -R_BP * X_EB_.translation();   // Base to Probe translation
    
    // Force transformation: f_base = R_BP * f_probe  
    ft_proc_b_.head<3>() = R_BP * ft_proc_s_.head<3>();
    
    // Torque transformation: tau_base = R_BP * tau_probe + [p_BP]× * R_BP * f_probe
    Vector3d cross_term = p_BP.cross(R_BP * ft_proc_s_.head<3>());
    ft_proc_b_.tail<3>() = R_BP * ft_proc_s_.tail<3>() + cross_term;
    
    // STEP 8: Publish compensated wrench in all three coordinate representations
    // 8a. Sensor frame (after bias/gravity compensation, before probe transformation)
    proc_msg_.header = msg->header;
    proc_msg_.header.frame_id = SENSOR_FRAME;
    // Convert Eigen 6D vector → ROS Wrench message for sensor frame output
    proc_msg_.wrench.force.x = ft_proc_s_[0]; proc_msg_.wrench.force.y = ft_proc_s_[1]; proc_msg_.wrench.force.z = ft_proc_s_[2];
    proc_msg_.wrench.torque.x = ft_proc_s_[3]; proc_msg_.wrench.torque.y = ft_proc_s_[4]; proc_msg_.wrench.torque.z = ft_proc_s_[5];
    wrench_proc_sensor_pub_->publish(proc_msg_);
    
    // 8b. Probe frame (same as processed sensor data in this case since sensor==probe)
    proc_msg_.header.frame_id = EE_FRAME;
    // Convert Eigen 6D vector → ROS Wrench message for probe frame output (same data as sensor)
    proc_msg_.wrench.force.x = ft_proc_s_[0]; proc_msg_.wrench.force.y = ft_proc_s_[1]; proc_msg_.wrench.force.z = ft_proc_s_[2];
    proc_msg_.wrench.torque.x = ft_proc_s_[3]; proc_msg_.wrench.torque.y = ft_proc_s_[4]; proc_msg_.wrench.torque.z = ft_proc_s_[5];
    wrench_proc_probe_pub_->publish(proc_msg_);
    
    // 8c. Base frame (transformed using adjoint matrix)
    proc_msg_.header.frame_id = BASE_FRAME;
    // Convert Eigen 6D vector → ROS Wrench message for base frame output (after spatial transform)
    proc_msg_.wrench.force.x = ft_proc_b_[0]; proc_msg_.wrench.force.y = ft_proc_b_[1]; proc_msg_.wrench.force.z = ft_proc_b_[2];
    proc_msg_.wrench.torque.x = ft_proc_b_[3]; proc_msg_.wrench.torque.y = ft_proc_b_[4]; proc_msg_.wrench.torque.z = ft_proc_b_[5];
    wrench_proc_probe_base_pub_->publish(proc_msg_);
}

bool WrenchNode::loadCalibrationParams() {
    // DESIGN: ROS2 parameter system allows runtime reconfiguration via launch files
    auto calib_file = declare_parameter<std::string>("calibration_file", 
        std::string(PACKAGE_SOURCE_DIR) + "/config/wrench_calibration.yaml");
    
    // CRITICAL: Fail fast if calibration missing - prevents undefined behavior
    if (!std::filesystem::exists(calib_file)) {
        RCLCPP_ERROR(get_logger(), "Calibration file not found");
        return false;
    }
    
    YAML::Node config = YAML::LoadFile(calib_file);
    
    // MATH: R_SE transforms sensor frame to end-effector frame (Yu et al. Eq. 6)
    auto rot_data = config["rotation_sensor_to_endeffector"].as<std::vector<std::vector<double>>>();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_SE_(i,j) = rot_data[i][j];  // Direct assignment more efficient than Eigen::Map
    
    // PHYSICS: These constants define the sensor-tool system for gravity compensation
    f_grav_b_ = Vector3d(config["gravity_in_base_frame"].as<std::vector<double>>().data());    // Gravity in robot base frame
    f_bias_s_ = Vector3d(config["force_bias"].as<std::vector<double>>().data());              // Sensor force offset
    t_bias_s_ = Vector3d(config["torque_bias"].as<std::vector<double>>().data());             // Sensor torque offset  
    p_CoM_s_ = Vector3d(config["tool_center_of_mass"].as<std::vector<double>>().data());      // Tool CoM for gravity torque calc
    
    RCLCPP_INFO(get_logger(), "Calibration loaded");
    return true;
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