#include "wrench_node.hpp"

namespace ur_admittance_controller {

// Implementation of ToMsg function declared in header
namespace conversions {
    geometry_msgs::msg::WrenchStamped ToMsg(
        const Wrench6d& wrench,
        const std::string& frame_id,
        const rclcpp::Time& stamp) {
        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.wrench.force.x = wrench[0];
        msg.wrench.force.y = wrench[1];
        msg.wrench.force.z = wrench[2];
        msg.wrench.torque.x = wrench[3];
        msg.wrench.torque.y = wrench[4];
        msg.wrench.torque.z = wrench[5];
        return msg;
    }
}

Matrix3d CrossMatrix(const Vector3d& v) {
    Matrix3d result;
    result <<     0, -v.z(),  v.y(),
              v.z(),      0, -v.x(),
             -v.y(),  v.x(),      0;
    return result;
}

Wrench6d TransformWrench(const Wrench6d& wrench_A, const Transform& X_BA) {
    Matrix3d R = X_BA.rotation();
    Vector3d p = X_BA.translation();

    Wrench6d wrench_B;
    wrench_B.head<3>() = R * wrench_A.head<3>();
    wrench_B.tail<3>() = R * wrench_A.tail<3>() + p.cross(wrench_B.head<3>());

    return wrench_B;
}

Wrench6d CompensateWrench(
    const Wrench6d& wrench_raw,
    const Transform& X_TB,
    const GravityCompensationParams& params) {

  Force3d f_gravity_S = params.R_SE * X_TB.rotation() * params.f_grav_b;

  Wrench6d wrench_gravity;
  wrench_gravity.head<3>() = f_gravity_S;
  wrench_gravity.tail<3>() = params.p_CoM_s.cross(f_gravity_S);

  Wrench6d wrench_bias;
  wrench_bias.head<3>() = params.f_bias_s;
  wrench_bias.tail<3>() = params.t_bias_s;

  return wrench_raw - wrench_gravity - wrench_bias;
}

Wrench6d TransformWrenchToBase(
    const Wrench6d& wrench_sensor,
    const Transform& X_BS) {
  return TransformWrench(wrench_sensor, X_BS.inverse());
}

WrenchNode::WrenchNode()
    : Node("wrench_node"),
      tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true)) {

    auto status = LoadCalibrationParams();
    if (!status) {
        RCLCPP_FATAL(get_logger(), "Failed to load calibration: %s - Calibration is required for safety",
                     status.error().message.c_str());
        std::exit(1);
    }

    SetupROSInterfaces();

    ComputeSensorToProbeAdjoint();

    RCLCPP_INFO(get_logger(), "Wrench node initialized with calibration");
}

void WrenchNode::SetupROSInterfaces() {
    wrench_sub_ = create_subscription<WrenchMsg>("/netft/raw_sensor", rclcpp::SensorDataQoS(),
        std::bind(&WrenchNode::WrenchCallback, this, std::placeholders::_1));
    wrench_proc_sensor_pub_ = create_publisher<WrenchMsg>(
        "/netft/proc_sensor", rclcpp::SensorDataQoS());
    wrench_proc_probe_pub_ = create_publisher<WrenchMsg>(
        "/netft/proc_probe", rclcpp::SensorDataQoS());
    wrench_proc_probe_base_pub_ = create_publisher<WrenchMsg>(
        "/netft/proc_probe_base", rclcpp::SensorDataQoS());
}

void WrenchNode::WrenchCallback(const WrenchMsg::ConstSharedPtr msg) {
    if (!tf_buffer_->canTransform(frames::ROBOT_TOOL_FRAME, frames::ROBOT_BASE_FRAME,
                                   tf2::TimePointZero)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
                             "Transform %s->%s unavailable", frames::ROBOT_BASE_FRAME, frames::ROBOT_TOOL_FRAME);
        return;
    }

    f_raw_s_ = SanitizeWrench(conversions::FromMsg(*msg));

    try {
        X_TB_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
            frames::ROBOT_TOOL_FRAME, frames::ROBOT_BASE_FRAME,
            tf2::TimePointZero, std::chrono::milliseconds(50)));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
                             "Transform lookup failed: %s", ex.what());
        return;
    }

    if (!X_TB_.matrix().allFinite()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
                             "Invalid transform (NaN/Inf)");
        return;
    }

    ft_proc_s_ = CompensateWrench(f_raw_s_, X_TB_, calibration_params_);
    wrench_probe_ = adjoint_probe_sensor_ * ft_proc_s_;

    Transform X_BP;
    try {
        X_BP = tf2::transformToEigen(tf_buffer_->lookupTransform(
            frames::ROBOT_BASE_FRAME, frames::PROBE_FRAME,
            tf2::TimePointZero, std::chrono::milliseconds(50)));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
                             "Transform lookup failed: %s", ex.what());
        return;
    }
    ft_proc_b_ = TransformWrenchToBase(wrench_probe_, X_BP);

    wrench_proc_sensor_pub_->publish(
        conversions::ToMsg(ft_proc_s_, frames::SENSOR_FRAME, msg->header.stamp));
    wrench_proc_probe_pub_->publish(
        conversions::ToMsg(wrench_probe_, frames::PROBE_FRAME, msg->header.stamp));
    wrench_proc_probe_base_pub_->publish(
        conversions::ToMsg(ft_proc_b_, frames::ROBOT_BASE_FRAME, msg->header.stamp));
}

Status WrenchNode::LoadCalibrationParams() {
    const char* workspace_env = std::getenv("ROS_WORKSPACE");
    std::string workspace = workspace_env ? workspace_env : 
                           std::string(std::getenv("HOME")) + "/ros2_ws";
    auto config_path = std::filesystem::path(workspace) / "src" / "ur_admittance_controller" / "config" / "wrench_calibration.yaml";
    
    if (!std::filesystem::exists(config_path)) {
        return tl::unexpected(MakeError(ErrorCode::kFileNotFound, 
            fmt::format("Config file not found: {}", config_path.string())));
    }
    
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path.string());
    } catch (const YAML::Exception& e) {
        return tl::unexpected(MakeError(ErrorCode::kInvalidConfiguration,
            fmt::format("Failed to parse YAML: {}", e.what())));
    }

    auto rot_data = config["sensor_rotation"].as<std::vector<double>>();
    
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_SE_(i,j) = rot_data[i*3 + j];

    auto gravity_vec = config["gravity_force"].as<std::vector<double>>();
    auto force_bias_vec = config["force_bias"].as<std::vector<double>>();
    auto torque_bias_vec = config["torque_bias"].as<std::vector<double>>();
    auto com_vec = config["center_of_mass"].as<std::vector<double>>();

    f_grav_b_ = Vector3d(gravity_vec.data());
    f_bias_s_ = Vector3d(force_bias_vec.data());
    t_bias_s_ = Vector3d(torque_bias_vec.data());
    p_CoM_s_ = Vector3d(com_vec.data());

    calibration_params_ = {R_SE_, f_grav_b_, f_bias_s_, t_bias_s_, p_CoM_s_};

    RCLCPP_INFO(get_logger(), "Calibration loaded");
    return {};
}

void WrenchNode::ComputeSensorToProbeAdjoint() {
    Eigen::Isometry3d X_SP;
    try {
        X_SP = tf2::transformToEigen(
            tf_buffer_->lookupTransform(frames::SENSOR_FRAME, frames::PROBE_FRAME,
                                       tf2::TimePointZero, std::chrono::seconds(5)));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "Transform %s->%s not available after 5s. Check URDF! Error: %s",
                     frames::SENSOR_FRAME, frames::PROBE_FRAME, ex.what());
        return;
    }

    Eigen::Matrix3d R_PS = X_SP.rotation().transpose();
    Eigen::Vector3d p_SP = X_SP.translation();

    adjoint_probe_sensor_.setZero();
    adjoint_probe_sensor_.block<3,3>(0,0) = R_PS;
    adjoint_probe_sensor_.block<3,3>(3,3) = R_PS;
    adjoint_probe_sensor_.block<3,3>(3,0) = -R_PS * CrossMatrix(p_SP);

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
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Failed to start wrench node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
