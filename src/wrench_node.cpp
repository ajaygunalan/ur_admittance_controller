// F/T sensor bias and gravity compensation node (Yu et al. method)
#include "wrench_node.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ur_admittance_controller {

// Frame names for ur_admittance_controller kinematic tree
static constexpr const char* BASE_FRAME = "base_link";
static constexpr const char* EE_FRAME = "tool0";
static constexpr const char* SENSOR_FRAME = "netft_link1";

WrenchNode::WrenchNode() : Node("wrench_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true))  // Enable tf2 caching for real-time lookup
{
    // Initialize all calibration parameters to safe defaults FIRST
    R_SE_ = Matrix3d::Identity();
    f_grav_b_ = Vector3d::Zero();
    f_bias_s_ = Vector3d::Zero();
    t_bias_s_ = Vector3d::Zero();
    p_CoM_s_ = Vector3d::Zero();
    
    // Initialize state variables
    f_raw_s_ = ft_proc_s_ = ft_proc_b_ = Wrench::Zero();  // 6D wrenches (force+torque)
    f_grav_s_ = Vector3d::Zero();                         // 3D gravity force only
    X_EB_ = Transform::Identity();                        // Robot pose transform
    
    // CRITICAL: Load calibration - throw on failure (Tier 2: setup error)
    auto status = loadCalibrationParams();
    if (!status) {
        RCLCPP_FATAL(get_logger(), "Failed to load calibration: %s", 
                     status.error().message.c_str());
        throw std::runtime_error("Calibration is required for safety - " + 
                                 status.error().message);
    }
    
    // Setup subscribers and publishers
    setupROSInterfaces();
    
    RCLCPP_INFO(get_logger(), "Wrench node initialized with calibration");
}

void WrenchNode::setupROSInterfaces() {
    // DESIGN: SensorDataQoS ensures reliable, real-time F/T data flow (~100-1000Hz)
    wrench_sub_ = create_subscription<WrenchMsg>("/netft/raw_sensor", rclcpp::SensorDataQoS(),
        [this](const WrenchMsg::ConstSharedPtr& msg) { wrench_callback(msg); });
    wrench_proc_sensor_pub_ = create_publisher<WrenchMsg>("/netft/proc_sensor", rclcpp::SensorDataQoS());
    wrench_proc_probe_pub_ = create_publisher<WrenchMsg>("/netft/proc_probe", rclcpp::SensorDataQoS());
    wrench_proc_probe_base_pub_ = create_publisher<WrenchMsg>("/netft/proc_probe_base", rclcpp::SensorDataQoS());
}

void WrenchNode::wrench_callback(const WrenchMsg::ConstSharedPtr msg) {
    // Tier 1: Critical invariant - tf_buffer must be initialized
    ENSURE(tf_buffer_ != nullptr, "TF buffer not initialized");
    
    if (!tf_buffer_->canTransform(EE_FRAME, BASE_FRAME, tf2::TimePointZero)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Transform not available from %s to %s - skipping wrench compensation",
                             EE_FRAME, BASE_FRAME);
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

Status WrenchNode::loadCalibrationParams() {
    // DESIGN: ROS2 parameter system allows runtime reconfiguration via launch files
    std::string default_calib_file;
    try {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("ur_admittance_controller");
        default_calib_file = package_share_dir + "/config/wrench_calibration.yaml";
    } catch (const std::exception& e) {
        return tl::unexpected(make_error(ErrorCode::kFileNotFound, 
                         "Package not found: " + std::string(e.what())));
    }
    
    auto calib_file = declare_parameter<std::string>("calibration_file", default_calib_file);
    
    // CRITICAL: Fail fast if calibration missing - prevents undefined behavior
    if (!std::filesystem::exists(calib_file)) {
        return tl::unexpected(make_error(ErrorCode::kFileNotFound,
                         "Calibration file not found: " + calib_file));
    }
    
    YAML::Node config;
    try {
        config = YAML::LoadFile(calib_file);
    } catch (const YAML::Exception& e) {
        return tl::unexpected(make_error(ErrorCode::kInvalidConfiguration,
                         "Failed to parse YAML file: " + std::string(e.what())));
    }
    
    // MATH: R_SE transforms sensor frame to end-effector frame (Yu et al. Eq. 6)
    auto rot_data = config["rotation_sensor_to_endeffector"].as<std::vector<std::vector<double>>>();
    
    // Tier 1: Rotation matrix must be 3x3
    ENSURE(rot_data.size() == 3 && rot_data[0].size() == 3 && rot_data[1].size() == 3 && rot_data[2].size() == 3,
           "Rotation matrix must be 3x3");
    
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_SE_(i,j) = rot_data[i][j];  // Direct assignment more efficient than Eigen::Map
    
    // PHYSICS: These constants define the sensor-tool system for gravity compensation
    auto gravity_vec = config["gravity_in_base_frame"].as<std::vector<double>>();
    auto force_bias_vec = config["force_bias"].as<std::vector<double>>();
    auto torque_bias_vec = config["torque_bias"].as<std::vector<double>>();
    auto com_vec = config["tool_center_of_mass"].as<std::vector<double>>();
    
    // Tier 1: All calibration vectors must be 3D
    ENSURE(gravity_vec.size() == 3 && force_bias_vec.size() == 3 && 
           torque_bias_vec.size() == 3 && com_vec.size() == 3,
           "All calibration vectors must have exactly 3 elements");
    
    f_grav_b_ = Vector3d(gravity_vec.data());    // Gravity in robot base frame
    f_bias_s_ = Vector3d(force_bias_vec.data());              // Sensor force offset
    t_bias_s_ = Vector3d(torque_bias_vec.data());             // Sensor torque offset  
    p_CoM_s_ = Vector3d(com_vec.data());      // Tool CoM for gravity torque calc
    
    RCLCPP_INFO(get_logger(), "Calibration loaded");
    return {};  // Success
}

}  // namespace ur_admittance_controller

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ur_admittance_controller::WrenchNode>();
        RCLCPP_INFO(node->get_logger(), "Wrench compensation node started");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), 
                     "Failed to start wrench node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}