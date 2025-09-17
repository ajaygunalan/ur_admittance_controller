#include "wrench_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ur_admittance_controller {

// Keep this lightweight conversion here (matches existing external interface)
namespace conversions {
geometry_msgs::msg::WrenchStamped ToMsg(const Wrench6d& w,
                                        const std::string& frame_id,
                                        const rclcpp::Time& stamp) {
  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.wrench.force.x  = w[0];
  msg.wrench.force.y  = w[1];
  msg.wrench.force.z  = w[2];
  msg.wrench.torque.x = w[3];
  msg.wrench.torque.y = w[4];
  msg.wrench.torque.z = w[5];
  return msg;
}
} // namespace conversions

// Gravity compensation (same math, concise)
Wrench6d CompensateWrench(const Wrench6d& w_raw,
                          const Transform& X_TB,
                          const GravityCompensationParams& p) {
  // g_S = R_SE * R_TB * g_B
  const Force3d  g_S = p.R_SE * X_TB.rotation() * p.f_grav_b;
  const Torque3d tau_g_S = p.p_CoM_s.cross(g_S);

  Wrench6d w_g;   w_g << g_S, tau_g_S;
  Wrench6d w_bias; w_bias << p.f_bias_s, p.t_bias_s;

  return w_raw - w_g - w_bias;
}

WrenchNode::WrenchNode()
: Node("wrench_node"),
  tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true)) {

  // Calibration must exist
  auto status = LoadCalibrationParams();
  if (!status) {
    RCLCPP_FATAL(get_logger(), "Failed to load calibration: %s", status.error().message.c_str());
    std::exit(1);
  }

  // Start timer to resolve static SENSOR→PROBE transform once spinning
  using namespace std::chrono_literals;
  init_timer_ = create_wall_timer(
      300ms, std::bind(&WrenchNode::InitStaticTransformOnce, this));
  SetupROSInterfaces();
  RCLCPP_INFO(get_logger(), "Wrench node initialized with calibration");
  RCLCPP_INFO(get_logger(), "Inspect raw wrench: ros2 topic echo /netft/raw_sensor --once");
  RCLCPP_INFO(get_logger(), "Inspect compensated sensor wrench: ros2 topic echo /netft/proc_sensor --once");
  RCLCPP_INFO(get_logger(), "Inspect probe-frame wrench: ros2 topic echo /netft/proc_probe --once");
  RCLCPP_INFO(get_logger(), "Initialize robot equilibrium pose: ros2 run ur_admittance_controller init_robot");
}

void WrenchNode::InitStaticTransformOnce() {
  std::string err;
  // Quiet check: retrieve error reason without tf2 printing warnings
  if (!tf_buffer_->canTransform(frames::PROBE_FRAME, frames::SENSOR_FRAME,
                                tf2::TimePointZero, &err)) {
    RCLCPP_DEBUG(get_logger(), "Waiting for static TF SENSOR->PROBE: %s", err.c_str());
    return;
  }
  T_SP_ = tf_buffer_->lookupTransform(frames::PROBE_FRAME, frames::SENSOR_FRAME,
                                      tf2::TimePointZero);
  sp_ready_ = true;
  init_timer_->cancel();
  RCLCPP_INFO(get_logger(), "Cached static SENSOR->PROBE transform");
}

void WrenchNode::SetupROSInterfaces() {
  wrench_sub_ = create_subscription<WrenchMsg>("/netft/raw_sensor", rclcpp::SensorDataQoS(),
      std::bind(&WrenchNode::WrenchCallback, this, std::placeholders::_1));

  wrench_proc_sensor_pub_ = create_publisher<WrenchMsg>("/netft/proc_sensor", rclcpp::SensorDataQoS());
  wrench_proc_probe_pub_  = create_publisher<WrenchMsg>("/netft/proc_probe",  rclcpp::SensorDataQoS());
  // /netft/proc_probe_base removed per request.
}

void WrenchNode::WrenchCallback(const WrenchMsg::ConstSharedPtr msg) {
  // Early return until static transform is ready
  if (!sp_ready_) return;

  // 1) LPF + sanitize (SENSOR frame)
  const Wrench6d w_raw  = conversions::FromMsg(*msg);
  if (first_sample_) { lpf_state_ = w_raw; first_sample_ = false; }
  const Wrench6d w_filt = ALPHA * w_raw + (1.0 - ALPHA) * lpf_state_;
  lpf_state_ = w_filt;
  const Wrench6d w_s    = SanitizeWrench(w_filt);

  // 2) TOOL ← BASE (dynamic) for gravity mapping; if missing, warn (throttled) and skip
  if (!tf_buffer_->canTransform(frames::ROBOT_TOOL_FRAME, frames::ROBOT_BASE_FRAME, tf2::TimePointZero)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
                         "TOOL←BASE transform not available");
    return;
  }
  const auto T_TB = tf_buffer_->lookupTransform(frames::ROBOT_TOOL_FRAME, frames::ROBOT_BASE_FRAME, tf2::TimePointZero);
  X_TB_ = tf2::transformToEigen(T_TB);

  // 3) Gravity + bias compensation (still in SENSOR)
  ft_proc_s_ = CompensateWrench(w_s, X_TB_, calibration_params_);

  // 4) Deadband (concise)
  for (int i = 0; i < 3; ++i) {
    if (std::abs(ft_proc_s_[i])   <= DEADBAND_FORCE)  ft_proc_s_[i]   = 0.0;
    if (std::abs(ft_proc_s_[i+3]) <= DEADBAND_TORQUE) ft_proc_s_[i+3] = 0.0;
  }

  // 5) Publish compensated SENSOR wrench
  wrench_proc_sensor_pub_->publish(conversions::ToMsg(ft_proc_s_, frames::SENSOR_FRAME, msg->header.stamp));

  // 6) Minimal, useful logging (throttled to 20 seconds)
  const double fN = ft_proc_s_.head<3>().norm(), tN = ft_proc_s_.tail<3>().norm();
  if (fN > constants::FORCE_THRESHOLD || tN > constants::TORQUE_THRESHOLD) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 20000,
      "Wrench[S]: F=%.2fN [%.2f,%.2f,%.2f], T=%.2fNm [%.2f,%.2f,%.2f]",
      fN, ft_proc_s_(0), ft_proc_s_(1), ft_proc_s_(2),
      tN, ft_proc_s_(3), ft_proc_s_(4), ft_proc_s_(5));
  }

  // 7) SENSOR → PROBE (static): full dual‑adjoint via tf2_kdl
  tf2::Stamped<KDL::Wrench> w_s_kdl, w_p_kdl;
  {
    const KDL::Vector F(ft_proc_s_(0), ft_proc_s_(1), ft_proc_s_(2));
    const KDL::Vector M(ft_proc_s_(3), ft_proc_s_(4), ft_proc_s_(5));
    w_s_kdl = tf2::Stamped<KDL::Wrench>(KDL::Wrench(F, M),
                                        tf2_ros::fromMsg(msg->header.stamp),
                                        frames::SENSOR_FRAME);
  }
  tf2::doTransform(w_s_kdl, w_p_kdl, T_SP_);

  auto probe_msg = tf2::toMsg(w_p_kdl);
  probe_msg.header.frame_id = frames::PROBE_FRAME;
  wrench_proc_probe_pub_->publish(probe_msg);
}

// Calibration load (same behavior; concise checks)
Status WrenchNode::LoadCalibrationParams() {
  const auto share_dir = ament_index_cpp::get_package_share_directory("ur_admittance_controller");
  const auto path = std::filesystem::path(share_dir) / "config" / "wrench_calibration.yaml";

  if (!std::filesystem::exists(path)) {
    return tl::unexpected(MakeError(ErrorCode::kFileNotFound,
           fmt::format("Config file not found: {}", path.string())));
  }

  YAML::Node cfg;
  try { cfg = YAML::LoadFile(path.string()); }
  catch (const YAML::Exception& e) {
    return tl::unexpected(MakeError(ErrorCode::kInvalidConfiguration,
           fmt::format("YAML parse error: {}", e.what())));
  }

  const auto rot = cfg["sensor_rotation"].as<std::vector<double>>();
  if (rot.size() != 9) {
    return tl::unexpected(MakeError(ErrorCode::kInvalidConfiguration,
           "sensor_rotation must have 9 elements"));
  }
  R_SE_ << rot[0], rot[1], rot[2],
           rot[3], rot[4], rot[5],
           rot[6], rot[7], rot[8];

  const auto grav  = cfg["gravity_force"].as<std::vector<double>>();
  const auto fbias = cfg["force_bias"].as<std::vector<double>>();
  const auto tbias = cfg["torque_bias"].as<std::vector<double>>();
  const auto com   = cfg["center_of_mass"].as<std::vector<double>>();
  if (grav.size()!=3 || fbias.size()!=3 || tbias.size()!=3 || com.size()!=3) {
    return tl::unexpected(MakeError(ErrorCode::kInvalidConfiguration,
           "gravity_force/force_bias/torque_bias/center_of_mass must be size 3"));
  }

  f_grav_b_ = Vector3d(grav.data());
  f_bias_s_ = Vector3d(fbias.data());
  t_bias_s_ = Vector3d(tbias.data());
  p_CoM_s_  = Vector3d(com.data());

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
