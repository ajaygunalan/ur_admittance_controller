// F/T sensor bias and gravity compensation node (Yu et al. method)
#include "wrench_node.hpp"
#include <ur_admittance_controller/utilities/conversions.hpp>
#include <ur_admittance_controller/utilities/spatial_math.hpp>
#include <ur_admittance_controller/algorithms/wrench_compensation.hpp>
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
    f_raw_s_ = conversions::fromMsg(*msg);
    
    // STEP 2: Get robot end-effector to base transform (X_EB)
    X_EB_ = tf2::transformToEigen(tf_buffer_->lookupTransform(EE_FRAME, BASE_FRAME, tf2::TimePointZero));
    
    // Tier 2: Validate external transform from TF2 (minimal sanity check)
    if (!X_EB_.matrix().allFinite()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Invalid transform from TF2 detected (contains NaN/Inf), skipping wrench update");
        return;  // Use last valid transform
    }
    
    // STEP 3: Apply compensation algorithm
    ft_proc_s_ = algorithms::compensateWrench(f_raw_s_, X_EB_, calibration_params_);
    
    // STEP 4: Transform to base frame
    Transform X_BS = X_EB_.inverse() * Transform(calibration_params_.R_SE);
    ft_proc_b_ = algorithms::transformWrenchToBase(ft_proc_s_, X_BS);
    
    // STEP 5: Publish compensated wrench in all three coordinate representations
    // 5a. Sensor frame (after bias/gravity compensation)
    auto sensor_msg = conversions::toMsg(ft_proc_s_, SENSOR_FRAME, msg->header.stamp);
    wrench_proc_sensor_pub_->publish(sensor_msg);
    
    // 5b. Probe frame (same as processed sensor data in this case since sensor==probe)
    auto probe_msg = conversions::toMsg(ft_proc_s_, EE_FRAME, msg->header.stamp);
    wrench_proc_probe_pub_->publish(probe_msg);
    
    // 5c. Base frame (transformed using adjoint matrix)
    auto base_msg = conversions::toMsg(ft_proc_b_, BASE_FRAME, msg->header.stamp);
    wrench_proc_probe_base_pub_->publish(base_msg);
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
    
    // Populate calibration params structure for algorithm
    calibration_params_.R_SE = R_SE_;
    calibration_params_.f_gravity_B = f_grav_b_;
    calibration_params_.f_bias_S = f_bias_s_;
    calibration_params_.t_bias_S = t_bias_s_;
    calibration_params_.p_SCoM_S = p_CoM_s_;
    
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