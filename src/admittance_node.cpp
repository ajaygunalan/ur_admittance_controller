// admittance_node.cpp — subscribes /netft/proc_probe (wrench in PROBE axes)
// Control loop order: FK → Admittance (v3) → PoseError(log) → Limits → IK

#include "admittance_node.hpp"

namespace ur_admittance_controller {

AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  RCLCPP_INFO_ONCE(get_logger(),
    "Initializing UR Admittance Controller (v3-fixed) — 6-DOF force-compliant control");

  param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
      get_node_parameters_interface());
  params_ = param_listener_->get_params();

  parameter_cb_handle_ = add_on_set_parameters_callback([this](const auto& params) {
    auto result = param_listener_->update(params);
    if (result.successful) {
      params_ = param_listener_->get_params();
      auto& p = params_.admittance;
      M_inverse_diag = Eigen::Map<const Eigen::VectorXd>(p.mass.data(), 6).cwiseInverse();
      K_diag = Eigen::Map<const Eigen::VectorXd>(p.stiffness.data(), 6);
      D_diag = Eigen::Map<const Eigen::VectorXd>(p.damping.data(), 6);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Admittance: M=[%s], K=[%s], D=[%s]",
        fmt::format("{:.1f}", fmt::join(p.mass, ", ")).c_str(),
        fmt::format("{:.1f}", fmt::join(p.stiffness, ", ")).c_str(),
        fmt::format("{:.1f}", fmt::join(p.damping, ", ")).c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Parameter update rejected: %s", result.reason.c_str());
    }
    return result;
  });

  auto& p = params_.admittance;
  M_inverse_diag = Eigen::Map<const Eigen::VectorXd>(p.mass.data(), 6).cwiseInverse();
  K_diag = Eigen::Map<const Eigen::VectorXd>(p.stiffness.data(), 6);
  D_diag = Eigen::Map<const Eigen::VectorXd>(p.damping.data(), 6);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "Admittance: M=[%s], K=[%s], D=[%s]",
    fmt::format("{:.1f}", fmt::join(p.mass, ", ")).c_str(),
    fmt::format("{:.1f}", fmt::join(p.stiffness, ", ")).c_str(),
    fmt::format("{:.1f}", fmt::join(p.damping, ", ")).c_str());

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
  X_BP_desired.linear() = Eigen::Quaterniond(ori[0], ori[1], ori[2], ori[3]).toRotationMatrix();

  logging::LogPose(get_logger(), "Equilibrium:",
                   Vector3d(pos.data()),
                   Eigen::Quaterniond(ori[0], ori[1], ori[2], ori[3]));
}

void AdmittanceNode::configure() {
  // Wrench in PROBE axes (matches refactored wrench_node)
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
  GetXBPCurrent();             // FK: X_BP_current, X_BW3
  ComputeAdmittance();         // v3-fixed core
  ComputePoseError();          // diagnostics (cmd − meas)
  LimitToWorkspace();
  ComputeAndPubJointVelocities();
}

void AdmittanceNode::WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  F_P_B = SanitizeWrench(conversions::FromMsg(*msg));  // keep as PROBE axes

  auto force_norm  = F_P_B.head<3>().norm();
  auto torque_norm = F_P_B.tail<3>().norm();
  if (force_norm > constants::FORCE_THRESHOLD || torque_norm > constants::TORQUE_THRESHOLD) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
      "Wrench[P]: F=%.2fN [%.2f,%.2f,%.2f], T=%.2fNm [%.2f,%.2f,%.2f]",
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
  // Orientation updates could be added similarly if you stream them.
  RCLCPP_DEBUG(get_logger(), "Desired position updated to [%.3f, %.3f, %.3f]",
               msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

}  

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