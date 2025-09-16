// wrench_node.cpp
#include "wrench_node.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ur_admittance_controller {

namespace conversions {
geometry_msgs::msg::WrenchStamped ToMsg(
    const Wrench6d& w, const std::string& frame_id, const rclcpp::Time& stamp) {
  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.wrench.force.x = w[0]; msg.wrench.force.y = w[1]; msg.wrench.force.z = w[2];
  msg.wrench.torque.x = w[3]; msg.wrench.torque.y = w[4]; msg.wrench.torque.z = w[5];
  return msg;
}
} // namespace conversions

WrenchNode::WrenchNode()
: Node("wrench_node"),
  tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true)) {

  auto status = LoadCalibrationParams();
  if (!status) {
    RCLCPP_FATAL(get_logger(), "Failed to load calibration: %s", status.error().message.c_str());
    std::exit(1);
  }

  // Static SENSOR -> PROBE transform (one-time)
  try {
    T_SP_ = tf_buffer_->lookupTransform(frames::PROBE_FRAME,
                                        frames::SENSOR_FRAME,
                                        tf2::TimePointZero,
                                        std::chrono::seconds(5));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_FATAL(get_logger(), "SENSOR->PROBE transform unavailable: %s", ex.what());
    std::exit(1);
  }

  SetupROSInterfaces();
  RCLCPP_INFO(get_logger(), "Wrench node initialized with calibration");
}

void WrenchNode::SetupROSInterfaces() {
  wrench_sub_ = create_subscription<WrenchMsg>("/netft/raw_sensor", rclcpp::SensorDataQoS(),
      std::bind(&WrenchNode::WrenchCallback, this, std::placeholders::_1));

  wrench_proc_sensor_pub_ = create_publisher<WrenchMsg>("/netft/proc_sensor", rclcpp::SensorDataQoS());
  wrench_proc_probe_pub_  = create_publisher<WrenchMsg>("/netft/proc_probe",  rclcpp::SensorDataQoS());
}

void WrenchNode::WrenchCallback(const WrenchMsg::ConstSharedPtr msg) {
  // Low-pass filter
  Wrench6d w_raw = conversions::FromMsg(*msg);
  if (first_sample_) { lpf_state_ = w_raw; first_sample_ = false; }
  Wrench6d w_filt = ALPHA * w_raw + (1.0 - ALPHA) * lpf_state_;
  lpf_state_ = w_filt;
  Wrench6d w_s = SanitizeWrench(w_filt); // SENSOR-frame, measured at SENSOR point

  // Dynamic TOOL <- BASE for gravity mapping (each callback)
  try {
    X_TB_ = tf2::transformToEigen(
              tf_buffer_->lookupTransform(frames::ROBOT_TOOL_FRAME,
                                          frames::ROBOT_BASE_FRAME,
                                          tf2::TimePointZero,
                                          std::chrono::milliseconds(50)));
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

  // Gravity + bias compensation in SENSOR
  ft_proc_s_ = CompensateWrench(w_s, X_TB_, calibration_params_);

  // Deadband
  for (int i = 0; i < 3; ++i) {
    if (std::abs(ft_proc_s_[i])   <= DEADBAND_FORCE)  ft_proc_s_[i]   = 0.0;
    if (std::abs(ft_proc_s_[i+3]) <= DEADBAND_TORQUE) ft_proc_s_[i+3] = 0.0;
  }

  // Publish compensated SENSOR wrench
  wrench_proc_sensor_pub_->publish(conversions::ToMsg(ft_proc_s_, frames::SENSOR_FRAME, msg->header.stamp));

  // Log norms when meaningful
  const double fN = ft_proc_s_.head<3>().norm(), tN = ft_proc_s_.tail<3>().norm();
  if (fN > constants::FORCE_THRESHOLD || tN > constants::TORQUE_THRESHOLD) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
      "Wrench: F=%.2fN [%.2f,%.2f,%.2f], T=%.2fNm [%.2f,%.2f,%.2f]",
      fN, ft_proc_s_(0), ft_proc_s_(1), ft_proc_s_(2),
      tN, ft_proc_s_(3), ft_proc_s_(4), ft_proc_s_(5));
  }

  // SENSOR -> PROBE (static): equivalent wrench at PROBE origin, expressed in PROBE
  tf2::Stamped<KDL::Wrench> w_s_kdl, w_p_kdl;
  {
    const auto& h = msg->header;
    const KDL::Vector F(ft_proc_s_(0), ft_proc_s_(1), ft_proc_s_(2));
    const KDL::Vector M(ft_proc_s_(3), ft_proc_s_(4), ft_proc_s_(5));
    w_s_kdl = tf2::Stamped<KDL::Wrench>(KDL::Wrench(F, M), tf2_ros::fromMsg(h.stamp), frames::SENSOR_FRAME);
  }

  tf2::doTransform(w_s_kdl, w_p_kdl, T_SP_); // full dual-adjoint: F' = R F; M' = R M + p×(R F)

  auto probe_msg = tf2::toMsg(w_p_kdl);
  probe_msg.header.frame_id = frames::PROBE_FRAME;
  wrench_proc_probe_pub_->publish(probe_msg);
}

Status WrenchNode::LoadCalibrationParams() {
  const char* workspace_env = std::getenv("ROS_WORKSPACE");
  std::string workspace = workspace_env ? workspace_env
                                        : std::string(std::getenv("HOME")) + "/ros2_ws";
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

  auto gravity_vec    = config["gravity_force"].as<std::vector<double>>();
  auto force_bias_vec = config["force_bias"].as<std::vector<double>>();
  auto torque_bias_vec= config["torque_bias"].as<std::vector<double>>();
  auto com_vec        = config["center_of_mass"].as<std::vector<double>>();

  f_grav_b_ = Vector3d(gravity_vec.data());
  f_bias_s_ = Vector3d(force_bias_vec.data());
  t_bias_s_ = Vector3d(torque_bias_vec.data());
  p_CoM_s_  = Vector3d(com_vec.data());

  calibration_params_ = {R_SE_, f_grav_b_, f_bias_s_, t_bias_s_, p_CoM_s_};
  RCLCPP_INFO(get_logger(), "Calibration loaded");
  return {};
}

} // namespace ur_admittance_controller

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
