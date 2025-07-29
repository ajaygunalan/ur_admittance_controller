// F/T sensor bias and gravity compensation node (Yu et al. method)
#include "wrench_node.hpp"
#include <ur_admittance_controller/utilities/conversions.hpp>
#include <ur_admittance_controller/utilities/spatial_math.hpp>
#include <ur_admittance_controller/algorithms/wrench_compensation.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace ur_admittance_controller {


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
    f_raw_s_ = ft_proc_s_ = wrench_probe = ft_proc_b_ = Wrench::Zero();  // 6D wrenches (force+torque)
    f_grav_s_ = Vector3d::Zero();                         // 3D gravity force only
    X_EB_ = Transform::Identity();                        // Robot pose transform
    adjoint_probe_sensor = Eigen::Matrix<double, 6, 6>::Identity();
    
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
    
    // CRITICAL: Must compute adjoint after TF is ready - this transforms 
    // forces/torques from sensor mounting point to actual tool contact point
    computeSensorToProbeAdjoint();
    
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
    
    if (!tf_buffer_->canTransform(frames::ROBOT_TOOL_FRAME, frames::ROBOT_BASE_FRAME, tf2::TimePointZero)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Transform not available from %s to %s - skipping wrench compensation",
                             frames::ROBOT_TOOL_FRAME, frames::ROBOT_BASE_FRAME);
        return;
    }
    
    // STEP 1: Extract raw F/T data from sensor (ROS WrenchStamped → Eigen 6D vector)
    // Apply sanitization to raw sensor data to handle floating-point noise
    f_raw_s_ = sanitizeWrench(conversions::fromMsg(*msg));
    
    // STEP 2: Get robot end-effector to base transform (X_EB)
    X_EB_ = tf2::transformToEigen(tf_buffer_->lookupTransform(frames::ROBOT_TOOL_FRAME, frames::ROBOT_BASE_FRAME, tf2::TimePointZero));
    
    // Tier 2: Validate external transform from TF2 (minimal sanity check)
    if (!X_EB_.matrix().allFinite()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Invalid transform from TF2 detected (contains NaN/Inf), skipping wrench update");
        return;  // Use last valid transform
    }
    
    // STEP 3: Apply Yu et al. compensation at sensor location
    ft_proc_s_ = algorithms::compensateWrench(f_raw_s_, X_EB_, calibration_params_);
    
    // STEP 4: Transform to probe tip - accounts for lever arm between sensor and tool
    wrench_probe = adjoint_probe_sensor * ft_proc_s_;
    
    // STEP 5: Express probe wrench in base coordinates for control algorithms
    Transform X_BP = tf2::transformToEigen(tf_buffer_->lookupTransform(
        frames::ROBOT_BASE_FRAME, frames::PROBE_FRAME, tf2::TimePointZero));
    ft_proc_b_ = algorithms::transformWrenchToBase(wrench_probe, X_BP);
    
    // STEP 6: Publish same physical wrench in three coordinate systems
    auto sensor_msg = conversions::toMsg(ft_proc_s_, frames::SENSOR_FRAME, msg->header.stamp);
    wrench_proc_sensor_pub_->publish(sensor_msg);
    
    auto probe_msg = conversions::toMsg(wrench_probe, frames::PROBE_FRAME, msg->header.stamp);
    wrench_proc_probe_pub_->publish(probe_msg);
    
    auto base_msg = conversions::toMsg(ft_proc_b_, frames::ROBOT_BASE_FRAME, msg->header.stamp);
    wrench_proc_probe_base_pub_->publish(base_msg);
}

Status WrenchNode::loadCalibrationParams() {
    // DESIGN: ROS2 parameter system allows runtime reconfiguration via launch files
    // Get workspace from environment or use default
    std::string workspace = std::getenv("ROS_WORKSPACE") ? 
                           std::getenv("ROS_WORKSPACE") : 
                           std::string(std::getenv("HOME")) + "/ros2_ws";
    std::string default_calib_file = workspace + "/src/ur_admittance_controller/config/wrench_calibration.yaml";
    
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
    
    // Populate calibration params structure for algorithm
    calibration_params_.R_SE = R_SE_;
    calibration_params_.f_gravity_B = f_grav_b_;
    calibration_params_.f_bias_S = f_bias_s_;
    calibration_params_.t_bias_S = t_bias_s_;
    calibration_params_.p_SCoM_S = p_CoM_s_;
    
    RCLCPP_INFO(get_logger(), "Calibration loaded");
    return {};  // Success
}

void WrenchNode::computeSensorToProbeAdjoint() {
    // Fatal if transform unavailable - we CANNOT publish wrong coordinate data
    if (!tf_buffer_->canTransform(frames::SENSOR_FRAME, frames::PROBE_FRAME, 
                                   tf2::TimePointZero, std::chrono::seconds(10))) {
        RCLCPP_FATAL(get_logger(), "Transform not available from %s to %s. "
                     "Check URDF - probe must be connected to sensor!",
                     frames::SENSOR_FRAME, frames::PROBE_FRAME);
        throw std::runtime_error("Required sensor-to-probe transform missing");
    }
    
    auto transform_msg = tf_buffer_->lookupTransform(
        frames::SENSOR_FRAME, frames::PROBE_FRAME, tf2::TimePointZero);
    Eigen::Isometry3d sensor_to_probe = tf2::transformToEigen(transform_msg);
    
    // Build 6x6 adjoint matrix for wrench transformation (Murray et al.)
    Eigen::Matrix3d R_PS = sensor_to_probe.rotation().transpose();
    Eigen::Vector3d p_SP = sensor_to_probe.translation();
    
    adjoint_probe_sensor = Eigen::Matrix<double, 6, 6>::Zero();
    adjoint_probe_sensor.block<3,3>(0,0) = R_PS;
    adjoint_probe_sensor.block<3,3>(3,3) = R_PS;
    adjoint_probe_sensor.block<3,3>(3,0) = -R_PS * spatial_math::skewSymmetric(p_SP);
    
    RCLCPP_INFO(get_logger(), "Sensor to probe transform ready");
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