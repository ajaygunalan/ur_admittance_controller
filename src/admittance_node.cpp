
/**
 * @file admittance_node.cpp
 * @brief ROS 2 node wiring. ControlCycle order now matches math dependencies:
 *
 *   ComputeAdmittance();     // Steps 0–5 (no FK)
 *   GetXBPCurrent();         // FK only for IK shift & logging
 *   ComputePoseError();      // cmd − meas (world)
 *   // add proportional correction in world and publish joint rates
 *
 * This removes the old coupling where admittance depended on current pose.
 * It keeps the external interface (topics/params) identical.
 */

#include "admittance_node.hpp"
#include "admittance_computations.hpp"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ur_admittance_controller {

namespace {
std::filesystem::path GetEquilibriumConfigPath() {
  const auto share_dir = ament_index_cpp::get_package_share_directory("ur_admittance_controller");
  return std::filesystem::path(share_dir) / "config" / "equilibrium.yaml";
}

constexpr std::array<double, 6> kDefaultVelocityGain{{4.0, 4.0, 4.0, 2.0, 2.0, 2.0}};
}

AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  RCLCPP_INFO_ONCE(get_logger(),
    "UR Admittance Controller — refactored to match pseudocode (factored adjoints, no FK in ODE).");

  param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
      get_node_parameters_interface());
  params_ = param_listener_->get_params();

  parameter_cb_handle_ = add_on_set_parameters_callback([this](const auto& params) {
    auto result = param_listener_->update(params);
    if (result.successful) {
      params_ = param_listener_->get_params();
      SetAdmittanceGains(params_.admittance);

      for (const auto& param : params) {
        if (param.get_name() != "tracking.velocity_gain") {
          continue;
        }

        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          RCLCPP_WARN(get_logger(),
                      "tracking.velocity_gain must be declared as an array of doubles; ignoring update");
          continue;
        }

        if (!UpdateWorldVelocityGain(param.as_double_array())) {
          RCLCPP_WARN(get_logger(),
                      "tracking.velocity_gain must have 6 elements; ignoring update");
        }
      }
    } else {
      RCLCPP_WARN(get_logger(), "Parameter update rejected: %s", result.reason.c_str());
    }
    return result;
  });

  SetAdmittanceGains(params_.admittance);

  const auto joint_count = params_.joints.size();
  q_current_.resize(joint_count, 0.0);
  q_dot_cmd_.resize(joint_count, 0.0);
  velocity_msg_.data.resize(joint_count);

  joint_name_to_index_.clear();
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_name_to_index_[params_.joints[i]] = i;
  }

  workspace_limits_ << constants::WORKSPACE_X_MIN, constants::WORKSPACE_X_MAX,
                       constants::WORKSPACE_Y_MIN, constants::WORKSPACE_Y_MAX,
                       constants::WORKSPACE_Z_MIN, constants::WORKSPACE_Z_MAX;
  arm_max_vel_ = 1.5;
  arm_max_acc_ = constants::ARM_MAX_ACCELERATION;
  admittance_ratio_ = 1.0;

  RCLCPP_INFO(get_logger(), "Admittance node constructed. Call configure() before spinning.");
}

void AdmittanceNode::SetAdmittanceGains(const Params::Admittance& p) {
  // Parameters remain in [linear; angular] order; state/order is consistent throughout.
  M_inverse_diag = Eigen::Map<const Eigen::VectorXd>(p.mass.data(), 6).cwiseInverse();
  K_diag = Eigen::Map<const Eigen::VectorXd>(p.stiffness.data(), 6);
  D_diag = Eigen::Map<const Eigen::VectorXd>(p.damping.data(), 6);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "Admittance: M=[%s], K=[%s], D=[%s]",
    fmt::format("{:.1f}", fmt::join(p.mass, ", ")).c_str(),
    fmt::format("{:.1f}", fmt::join(p.stiffness, ", ")).c_str(),
    fmt::format("{:.1f}", fmt::join(p.damping, ", ")).c_str());
}

void AdmittanceNode::configure() {
  const auto config_path = GetEquilibriumConfigPath();
  LoadEquilibriumPose(config_path);

  const std::vector<double> kv_default{kDefaultVelocityGain.begin(), kDefaultVelocityGain.end()};
  if (!has_parameter("tracking.velocity_gain")) {
    declare_parameter("tracking.velocity_gain", kv_default);
  }

  std::vector<double> kv_user = kv_default;
  get_parameter("tracking.velocity_gain", kv_user);
  if (!UpdateWorldVelocityGain(kv_user)) {
    RCLCPP_WARN(get_logger(),
                "tracking.velocity_gain must have 6 elements; reverting to defaults");
    UpdateWorldVelocityGain(kv_default);
    set_parameter(rclcpp::Parameter("tracking.velocity_gain", kv_default));
  }

  // Wrench topic — per README, this should be gravity-compensated and in WORLD/base.
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/netft/proc_probe", rclcpp::SensorDataQoS(),
      std::bind(&AdmittanceNode::WrenchCallback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,
      std::bind(&AdmittanceNode::JointStateCallback, this, std::placeholders::_1));

  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/admittance_node/desired_pose", constants::DEFAULT_QUEUE_SIZE,
      std::bind(&AdmittanceNode::DesiredPoseCallback, this, std::placeholders::_1));

  velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", constants::DEFAULT_QUEUE_SIZE);

  if (auto status = LoadKinematics(); !status) {
    RCLCPP_FATAL(get_logger(), "Failed to load kinematics: %s", status.error().message.c_str());
    std::exit(1);
  }

  control_timer_ = create_wall_timer(
      std::chrono::nanoseconds(control_period_.nanoseconds()),
      std::bind(&AdmittanceNode::OnControlTimer, this));

  RCLCPP_INFO(get_logger(),
              "Configured: equilibrium=[%.3f, %.3f, %.3f], loop=%d Hz, config=%s",
              X_BP_desired.translation()(0),
              X_BP_desired.translation()(1),
              X_BP_desired.translation()(2),
              constants::CONTROL_LOOP_HZ,
              config_path.string().c_str());
  RCLCPP_INFO(get_logger(), "Waiting for initial joint states...");
}

void AdmittanceNode::LoadEquilibriumPose(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    RCLCPP_FATAL(get_logger(), "Equilibrium config not found: %s", path.string().c_str());
    std::exit(1);
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(path.string());
  } catch (const YAML::Exception& e) {
    RCLCPP_FATAL(get_logger(), "Failed to parse equilibrium file %s: %s",
                 path.string().c_str(), e.what());
    std::exit(1);
  }

  auto eq_params = config["admittance_node"]["ros__parameters"];
  const auto position = eq_params["equilibrium.position"].as<std::vector<double>>();
  const auto orientation = eq_params["equilibrium.orientation"].as<std::vector<double>>();

  if (position.size() != 3 || orientation.size() != 4) {
    RCLCPP_FATAL(get_logger(),
                 "Equilibrium pose must contain 3 position elements and 4 orientation elements");
    std::exit(1);
  }

  const Eigen::Vector3d pos_vec(position[0], position[1], position[2]);
  X_BP_desired.translation() = pos_vec;
  Eigen::Quaterniond q_des(orientation[0], orientation[1], orientation[2], orientation[3]);
  q_des.normalize();
  X_BP_desired.linear() = q_des.toRotationMatrix();

  g_X_WB_cmd = X_BP_desired;

  logging::LogPose(get_logger(), "Equilibrium:", pos_vec, q_des);
}

bool AdmittanceNode::UpdateWorldVelocityGain(const std::vector<double>& gains) {
  if (gains.size() != 6) {
    return false;
  }
  world_vel_P_ = Eigen::Map<const Vector6d>(gains.data());
  return true;
}

void AdmittanceNode::OnControlTimer() {
  if (!joint_states_received_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
                          "Waiting for joint states before running control loop");
    return;
  }
  ControlCycle();
}

void AdmittanceNode::ControlCycle() {
  // New order: math first (purely from desired pose + wrench), then FK for IK & error.
  ComputeAdmittance();         // Steps 0–5, independent of FK
  GetXBPCurrent();             // FK: X_BP_current, X_BW3
  ComputePoseError();          // diagnostics (cmd − meas)

  // Add proportional world correction now that e_W is known (gain units 1/s).
  V_P_B_commanded += world_vel_P_.cwiseProduct(X_BP_error);

  LimitToWorkspace();
  ComputeAndPubJointVelocities();
}

void AdmittanceNode::WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  // Store the latest wrench (assumed WORLD/base).
  F_P_B = SanitizeWrench(conversions::FromMsg(*msg));

  auto force_norm  = F_P_B.head<3>().norm();
  auto torque_norm = F_P_B.tail<3>().norm();
  if (force_norm > constants::FORCE_THRESHOLD || torque_norm > constants::TORQUE_THRESHOLD) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
      "Wrench[W]: F=%.2fN [%.2f,%.2f,%.2f], T=%.2fNm [%.2f,%.2f,%.2f]",
      force_norm, F_P_B(0), F_P_B(1), F_P_B(2),
      torque_norm, F_P_B(3), F_P_B(4), F_P_B(5));
  }
}

void AdmittanceNode::JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(), "Joint states received - robot is online");
  joint_states_received_ = true;

  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (auto it = joint_name_to_index_.find(msg->name[i]); it != joint_name_to_index_.end()) {
      q_current_[it->second] = SanitizeJointAngle(msg->position[i]);
    }
  }
}

void AdmittanceNode::DesiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  X_BP_desired.translation() << msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z;
  const auto& q = msg->pose.orientation;
  Eigen::Quaterniond q_new(q.w, q.x, q.y, q.z);
  if (q_new.norm() > 1e-6) {
    X_BP_desired.linear() = q_new.normalized().toRotationMatrix();
  }

  RCLCPP_DEBUG(get_logger(), "Desired pose updated. p=[%.3f, %.3f, %.3f]",
               msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

}  // namespace ur_admittance_controller

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  node->configure();

  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "Shutting down admittance controller...");
  rclcpp::shutdown();
  return 0;
}
