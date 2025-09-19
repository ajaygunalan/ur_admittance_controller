#include "admittance_node.hpp"
#include "admittance_computations.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/core.h>

namespace ur_admittance_controller {

namespace {
std::filesystem::path GetEquilibriumConfigPath() {
  const auto share_dir = ament_index_cpp::get_package_share_directory("ur_admittance_controller");
  return std::filesystem::path(share_dir) / "config" / "equilibrium.yaml";
}
} // namespace

AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
      get_node_parameters_interface());
  params_ = param_listener_->get_params();

  parameter_cb_handle_ = add_on_set_parameters_callback([this](const auto& params) {
    auto result = param_listener_->update(params);
    if (result.successful) {
      params_ = param_listener_->get_params();
      SetAdmittanceGains(params_.admittance);
      {
        const auto& kv = params_.tracking.velocity_gain;
        UpdateWorldVelocityGain(std::vector<double>(kv.begin(), kv.end()));
      }
    } else {
      RCLCPP_WARN(get_logger(), "Parameter update rejected: %s", result.reason.c_str());
    }
    return result;
  });

  SetAdmittanceGains(params_.admittance);
  {
    const auto& kv = params_.tracking.velocity_gain;
    UpdateWorldVelocityGain(std::vector<double>(kv.begin(), kv.end()));
  }

  const auto joint_count = params_.joints.size();
  q_current_.resize(joint_count, 0.0);
  q_dot_cmd_.resize(joint_count, 0.0);
  velocity_msg_.data.resize(joint_count);

  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_name_to_index_[params_.joints[i]] = i;
  }

  workspace_limits_ << constants::WORKSPACE_X_MIN, constants::WORKSPACE_X_MAX,
                       constants::WORKSPACE_Y_MIN, constants::WORKSPACE_Y_MAX,
                       constants::WORKSPACE_Z_MIN, constants::WORKSPACE_Z_MAX;

  RCLCPP_INFO(get_logger(), "Admittance node constructed. Call configure() before spinning.");
}

void AdmittanceNode::SetAdmittanceGains(const Params::Admittance& a) {
  M_inv_diag_ = Eigen::Map<const Eigen::VectorXd>(a.mass.data(), 6).cwiseInverse();
  D_diag_     = Eigen::Map<const Eigen::VectorXd>(a.damping.data(), 6);
  K_diag_     = Eigen::Map<const Eigen::VectorXd>(a.stiffness.data(), 6);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "M=[%s], D=[%s], K=[%s]",
    fmt::format("{:.1f}", fmt::join(a.mass, ", ")).c_str(),
    fmt::format("{:.1f}", fmt::join(a.damping, ", ")).c_str(),
    fmt::format("{:.1f}", fmt::join(a.stiffness, ", ")).c_str());
}

void AdmittanceNode::configure() {
  const auto cfg_path = GetEquilibriumConfigPath();
  LoadEquilibriumPose(cfg_path);

  // Inputs
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/netft/proc_probe", rclcpp::SensorDataQoS(),
      std::bind(&AdmittanceNode::WrenchCallback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1, std::bind(&AdmittanceNode::JointStateCallback, this, std::placeholders::_1));

  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/admittance_node/desired_pose", constants::DEFAULT_QUEUE_SIZE,
      std::bind(&AdmittanceNode::DesiredPoseCallback, this, std::placeholders::_1));

  // Output
  velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", constants::DEFAULT_QUEUE_SIZE);

  if (auto status = LoadKinematics(); !status) {
    RCLCPP_FATAL(get_logger(), "Kinematics init failed: %s", status.error().message.c_str());
    std::exit(1);
  }

  control_timer_ = create_wall_timer(
      std::chrono::nanoseconds(control_period_.nanoseconds()),
      std::bind(&AdmittanceNode::OnControlTimer, this));

  RCLCPP_INFO(get_logger(), "Configured; loop=%d Hz, config=%s",
              constants::CONTROL_LOOP_HZ, cfg_path.string().c_str());
}

void AdmittanceNode::LoadEquilibriumPose(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    RCLCPP_FATAL(get_logger(), "Equilibrium config not found: %s", path.string().c_str());
    std::exit(1);
  }
  YAML::Node config = YAML::LoadFile(path.string());
  auto eq = config["admittance_node"]["ros__parameters"];
  const auto pos = eq["equilibrium.position"].as<std::vector<double>>();
  const auto ori = eq["equilibrium.orientation"].as<std::vector<double>>();
  if (pos.size()!=3 || ori.size()!=4) {
    RCLCPP_FATAL(get_logger(), "Equilibrium pose malformed in %s", path.string().c_str());
    std::exit(1);
  }
  X_WP_desired_.translation() = Vector3d(pos[0], pos[1], pos[2]);
  Eigen::Quaterniond q_des(ori[0], ori[1], ori[2], ori[3]);
  q_des.normalize();
  X_WP_desired_.linear() = q_des.toRotationMatrix();
  X_WP_cmd_ = X_WP_desired_;
  logging::LogPose(get_logger(), "Equilibrium", X_WP_desired_.translation(), q_des);
}

bool AdmittanceNode::UpdateWorldVelocityGain(const std::vector<double>& gains) {
  if (gains.size() != 6) return false;
  Kp_W_ = Eigen::Map<const Vector6d>(gains.data());
  return true;
}

void AdmittanceNode::OnControlTimer() {
  if (!joint_states_received_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
                          "Waiting for joint states");
    return;
  }
  ControlCycle();
}

void AdmittanceNode::ControlCycle() {
  ComputeAdmittance();     // 0–5
  GetXWPCurrent();
  ComputePoseError();

  // 5) Twist = (planner/desired, here zero) + admittance_rate + Kp * error
  V_cmd_W_ = V_cmd_W_ + Kp_W_.cwiseProduct(X_WP_error_); // V_cmd_W_ already holds admittance_rate

  LimitWorldCommand();
  ComputeAndPubJointVelocities();
}

void AdmittanceNode::ComputeAdmittance() {
  const double dt = control_period_.seconds();
  const Matrix3d R_des = X_WP_desired_.rotation();
  const Vector3d p_des = X_WP_desired_.translation();

  // 0) F_ext_P_: already PROBE frame
  const Vector6d F_P = F_ext_P_ * admittance_ratio_;

  // 1) ODE in P
  const Vector6d acc_P = ComputeAdmittanceAcceleration(
      F_P, deltaXdot_P_, deltaX_P_, M_inv_diag_, D_diag_, K_diag_);

  deltaXdot_P_ = IntegrateVelocity(deltaXdot_P_, acc_P, dt);
  deltaX_P_    = IntegrateVelocity(deltaX_P_,    deltaXdot_P_, dt);

  // 2) Map offset & rate to WORLD (desired pose anchors the axes)
  const Adjoints A = BuildAdjointsFromDesired(R_des, p_des);
  const Vector6d dX_W    = A.Ad_pose  * deltaX_P_;
  const Vector6d dXdot_W = A.Ad_twist * deltaXdot_P_;

  // 3) Compose commanded pose in WORLD
  const Vector3d drot = dX_W.tail<3>();
  const double th = drot.norm();
  Matrix3d R_cmd = (th < 1e-12) ? R_des : (Eigen::AngleAxisd(th, drot / th).toRotationMatrix() * R_des);

  X_WP_cmd_ = Eigen::Isometry3d::Identity();
  X_WP_cmd_.linear() = R_cmd;
  X_WP_cmd_.translation() = p_des + dX_W.head<3>();

  // 4) error uses FK; prepare open-loop world twist (planner term is zero here)
  V_cmd_W_.setZero();
  V_cmd_W_.head<3>() = dXdot_W.head<3>();
  V_cmd_W_.tail<3>() = dXdot_W.tail<3>();
}

void AdmittanceNode::GetXWPCurrent() {
  auto result = ComputeForwardKinematics(q_current_, fk_pos_solver_.get(), X_W3P_);
  if (!result) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
                          "FK failed: %s", result.error().message.c_str());
    return;
  }
  X_WP_current_ = result.value();

  for (size_t i = 0; i < num_joints_; ++i) q_kdl_(i) = q_current_[i];
  fk_pos_solver_->JntToCart(q_kdl_, X_BW3_);
}

void AdmittanceNode::ComputePoseError() {
  X_WP_error_ = ::ur_admittance_controller::ComputePoseError(X_WP_cmd_, X_WP_current_);
  auto [pos_err, ori_err] = ::ur_admittance_controller::GetPoseErrorNorms(X_WP_error_);
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
    "err pos=%.4f, ori=%.4f", pos_err, ori_err);
}

void AdmittanceNode::LimitWorldCommand() {
  const double dt = control_period_.seconds();

  // a) Acceleration-rate gating in WORLD (memory V_prev_W_)
  Vector6d V_minus = V_cmd_W_;
  Vector3d dlin = V_minus.head<3>() - V_prev_W_.head<3>();
  Vector3d dang = V_minus.tail<3>() - V_prev_W_.tail<3>();

  const double max_dlin = arm_max_acc_ * dt;
  const double max_dang = params_.admittance.limits.max_angular_acceleration * dt;

  const double nlin = dlin.norm();
  if (nlin > max_dlin && nlin > 1e-12) dlin *= (max_dlin / nlin);
  const double nang = dang.norm();
  if (nang > max_dang && nang > 1e-12) dang *= (max_dang / nang);

  Vector6d V_a = V_prev_W_;
  V_a.head<3>() += dlin;
  V_a.tail<3>() += dang;

  // b) Workspace wall gating (use measured TCP position)
  V_a = ApplyWorkspaceLimits(V_a, X_WP_current_.translation(), workspace_limits_);

  // c) Velocity norm caps
  V_cmd_W_ = LimitVelocityMagnitude(
      V_a, arm_max_vel_, params_.admittance.limits.max_angular_velocity);

  V_prev_W_ = V_cmd_W_;
}

void AdmittanceNode::ComputeAndPubJointVelocities() {
  auto result = ComputeInverseKinematicsVelocity(
      V_cmd_W_, X_WP_current_, X_BW3_, q_current_, ik_vel_solver_.get());

  if (!result) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
        "IK failed: v=[%.3f,%.3f,%.3f], w=[%.3f,%.3f,%.3f]",
        V_cmd_W_(0), V_cmd_W_(1), V_cmd_W_(2),
        V_cmd_W_(3), V_cmd_W_(4), V_cmd_W_(5));
    std::fill(q_dot_cmd_.begin(), q_dot_cmd_.end(), 0.0);
  } else {
    q_dot_cmd_ = result.value();
  }

  velocity_msg_.data = q_dot_cmd_;
  velocity_pub_->publish(velocity_msg_);
}

void AdmittanceNode::WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  // NOTE:
  // The wrench published on /netft/proc_probe follows the sensor/reaction convention
  // (robot-on-world). The admittance ODE expects the external wrench applied TO the robot
  // (environment-on-robot). Flip sign once at ingestion to match that convention.
  F_ext_P_ = -conversions::FromMsg(*msg);
  const double fN = F_ext_P_.head<3>().norm();
  const double tN = F_ext_P_.tail<3>().norm();
  if (fN > constants::FORCE_THRESHOLD || tN > constants::TORQUE_THRESHOLD) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
      "wrench P: |F|=%.2fN, |T|=%.2fNm", fN, tN);
  }
}

void AdmittanceNode::JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(), "Joint states online");
  joint_states_received_ = true;

  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (auto it = joint_name_to_index_.find(msg->name[i]); it != joint_name_to_index_.end()) {
      q_current_[it->second] = msg->position[i];
    }
  }
}

void AdmittanceNode::DesiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  X_WP_desired_.translation() << msg->pose.position.x,
                                 msg->pose.position.y,
                                 msg->pose.position.z;
  const auto& q = msg->pose.orientation;
  Eigen::Quaterniond q_new(q.w, q.x, q.y, q.z);
  if (q_new.norm() > 1e-6) {
    X_WP_desired_.linear() = q_new.normalized().toRotationMatrix();
  }
  RCLCPP_DEBUG(get_logger(), "desired pose updated");
}

Status AdmittanceNode::LoadKinematics() {
  RCLCPP_INFO(get_logger(), "Loading kinematics from robot_description");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/robot_state_publisher");
  if (!parameters_client->wait_for_service(constants::DEFAULT_TIMEOUT)) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed, "robot_state_publisher not available"));
  }
  auto parameters = parameters_client->get_parameters({"robot_description"});
  if (parameters.empty() || parameters[0].get_type() == rclcpp::PARAMETER_NOT_SET) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed, "robot_description not set"));
  }
  urdf::Model urdf_model;
  if (!urdf_model.initString(parameters[0].as_string())) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed, "URDF parse failed"));
  }

  auto result = kinematics::InitializeFromUrdf(urdf_model, "base_link", "p42v_link1");
  if (!result) return tl::unexpected(result.error());

  auto& c = result.value();
  kdl_tree_  = std::move(c.tree);
  kdl_chain_ = c.robot_chain;
  X_W3P_     = c.probe_offset;

  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_, 1e-5, 150);
  ik_vel_solver_->setLambda(0.1);

  num_joints_ = kdl_chain_.getNrOfJoints();
  q_kdl_.resize(num_joints_);
  v_kdl_.resize(num_joints_);
  return {};
}

} // namespace ur_admittance_controller

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  node->configure();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
