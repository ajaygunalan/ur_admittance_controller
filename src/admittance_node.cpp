
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
 * It keeps the external interface (topics/params) identical.                     fileciteturn0file5
 */

#include "admittance_node.hpp"
#include "admittance_computations.hpp"

namespace ur_admittance_controller {

namespace {
// World-frame velocity gain (1/s) multiplying pose error.
Vector6d g_world_vel_P = (Vector6d() << 4.0, 4.0, 4.0, 2.0, 2.0, 2.0).finished();
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
        if (param.get_name() == "tracking.velocity_gain" &&
            param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          const auto& values = param.as_double_array();
          if (values.size() == 6) {
            g_world_vel_P = Eigen::Map<const Vector6d>(values.data());
          } else {
            RCLCPP_WARN(get_logger(),
                        "tracking.velocity_gain must have 6 elements; ignoring update");
          }
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

  // Load equilibrium (desired) pose
  const char* workspace_env = std::getenv("ROS_WORKSPACE");
  std::string workspace = workspace_env ? workspace_env :
                         std::string(std::getenv("HOME")) + "/ros2_ws";
  auto config_path = std::filesystem::path(workspace) / "src" / "ur_admittance_controller" / "config" / "equilibrium.yaml";

  if (!std::filesystem::exists(config_path)) {
    RCLCPP_FATAL(get_logger(), "Failed to load equilibrium file: %s", config_path.string().c_str());
    std::exit(1);
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path.string());
  } catch (const YAML::Exception& e) {
    RCLCPP_FATAL(get_logger(), "Failed to parse equilibrium file: %s", e.what());
    std::exit(1);
  }

  auto eq_params = config["admittance_node"]["ros__parameters"];
  auto pos = eq_params["equilibrium.position"].as<std::vector<double>>();
  auto ori = eq_params["equilibrium.orientation"].as<std::vector<double>>();
  X_BP_desired.translation() << pos[0], pos[1], pos[2];
  X_BP_desired.linear() = Eigen::Quaterniond(ori[0], ori[1], ori[2], ori[3]).normalized().toRotationMatrix();

  const std::vector<double> kv_default = {4.0, 4.0, 4.0, 2.0, 2.0, 2.0};
  declare_parameter<std::vector<double>>("tracking.velocity_gain", kv_default);
  std::vector<double> kv_user;
  if (get_parameter("tracking.velocity_gain", kv_user)) {
    if (kv_user.size() == 6) {
      g_world_vel_P = Eigen::Map<const Vector6d>(kv_user.data());
    } else {
      RCLCPP_WARN(get_logger(),
                  "tracking.velocity_gain must have 6 elements; using defaults");
    }
  }

  logging::LogPose(get_logger(), "Equilibrium:",
                   Vector3d(pos.data()),
                   Eigen::Quaterniond(ori[0], ori[1], ori[2], ori[3]));
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
  // Wrench topic — per README, this should be gravity-compensated and in WORLD/base.  fileciteturn0file5
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/netft/proc_probe", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg) {
        WrenchCallback(msg);
      });

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        JointStateCallback(msg);
      });

  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/admittance_node/desired_pose", constants::DEFAULT_QUEUE_SIZE,
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
        DesiredPoseCallback(msg);
      });

  velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", constants::DEFAULT_QUEUE_SIZE);

  if (auto status = LoadKinematics(); !status) {
    RCLCPP_FATAL(get_logger(), "Failed to load kinematics: %s", status.error().message.c_str());
    std::exit(1);
  }

  RCLCPP_INFO(get_logger(),
    "Configured: equilibrium=[%.3f, %.3f, %.3f], control=100Hz",
    X_BP_desired.translation()(0), X_BP_desired.translation()(1), X_BP_desired.translation()(2));
}

void AdmittanceNode::ControlCycle() {
  // New order: math first (purely from desired pose + wrench), then FK for IK & error.
  ComputeAdmittance();         // Steps 0–5, independent of FK
  GetXBPCurrent();             // FK: X_BP_current, X_BW3
  ComputePoseError();          // diagnostics (cmd − meas)

  // Add proportional world correction now that e_W is known (gain units 1/s).
  V_P_B_commanded += g_world_vel_P.cwiseProduct(X_BP_error);

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

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  rclcpp::Rate loop_rate(ur_admittance_controller::constants::CONTROL_LOOP_HZ);
  RCLCPP_INFO(node->get_logger(), "Waiting for initial joint states...");

  while (rclcpp::ok()) {
    executor.spin_some();
    if (node->joint_states_received_) {
      RCLCPP_INFO_ONCE(node->get_logger(), "Joint states received. Running control at 100Hz...");
      node->ControlCycle();
    }
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Shutting down admittance controller...");
  rclcpp::shutdown();
  return 0;
}
