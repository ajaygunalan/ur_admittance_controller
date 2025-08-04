#include "admittance_node.hpp"
#include <utilities/logging.hpp>
#include <utilities/constants.hpp>

namespace ur_admittance_controller {


// Helper function to map joint states from JointState message to internal vector
void AdmittanceNode::MapJointStates(const sensor_msgs::msg::JointState& msg) {
  // Map joint states using pre-built member mapping
  for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
    if (auto it = joint_name_to_index_.find(msg.name[i]); it != joint_name_to_index_.end()) {
      // Drake-inspired sanitization to handle floating-point noise
      q_current_[it->second] = SanitizeJointAngle(msg.position[i]);
    }
    // Silently ignore joints not in our kinematic chain (e.g., ft_sensor_joint)
    // These are intentionally present in the URDF for simulation purposes
  }
}

void AdmittanceNode::InitializeParameters() {
  param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
      get_node_parameters_interface());
  params_ = param_listener_->get_params();

  parameter_cb_handle_ = add_on_set_parameters_callback([this](const auto& params) {
    auto result = param_listener_->update(params);
    if (result.successful) {
      params_ = param_listener_->get_params();
      UpdateAdmittanceParameters();
    } else {
      RCLCPP_WARN(get_logger(), "Parameter update rejected: %s", result.reason.c_str());
    }
    return result;
  });

  UpdateAdmittanceParameters();
}

void AdmittanceNode::SetupROSInterfaces() {
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/netft/proc_probe_base", rclcpp::SensorDataQoS(),
      std::bind(&AdmittanceNode::WrenchCallback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,
      std::bind(&AdmittanceNode::JointStateCallback, this, std::placeholders::_1));

  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/admittance_node/desired_pose", constants::DEFAULT_QUEUE_SIZE,
      std::bind(&AdmittanceNode::DesiredPoseCallback, this, std::placeholders::_1));

  velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", constants::DEFAULT_QUEUE_SIZE);
}




void AdmittanceNode::WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
  F_P_B = SanitizeWrench(conversions::FromMsg(*msg));

  // Throttled logging for significant forces (>0.1N or >0.1Nm)
  auto [force_norm, torque_norm] = std::make_pair(F_P_B.head<3>().norm(), F_P_B.tail<3>().norm());
  if (force_norm > constants::FORCE_THRESHOLD || torque_norm > constants::TORQUE_THRESHOLD) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
      "Wrench: F=%.2fN [%.2f,%.2f,%.2f], T=%.2fNm [%.2f,%.2f,%.2f]",
      force_norm, F_P_B(0), F_P_B(1), F_P_B(2),
      torque_norm, F_P_B(3), F_P_B(4), F_P_B(5));
  }
}

void AdmittanceNode::JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(), "Joint states received - robot is online");
  joint_states_received_ = true;
  MapJointStates(*msg);
}

// Handle desired pose updates from ROS2 topic
void AdmittanceNode::DesiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  // Only update position, keep orientation fixed at equilibrium
  X_BP_desired.translation() << msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z;

  // Orientation remains fixed at equilibrium value

  RCLCPP_DEBUG(get_logger(), "Desired position updated to [%.3f, %.3f, %.3f]",
               msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void AdmittanceNode::ControlCycle() {
  if (!joint_states_received_) {
    RCLCPP_INFO_ONCE(get_logger(), "Waiting for initial joint states...");
    return;
  }

  GetXBPCurrent();
  ComputePoseError();
  ComputeAdmittance();
  LimitToWorkspace();
  ComputeAndPubJointVelocities();
}


void AdmittanceNode::Initialize() {

  // Tier 2: Throw on setup failure
  if (auto status = LoadKinematics(); !status) {
    throw std::runtime_error(status.error().message);
  }

  kinematics_initialized_ = true;

  joint_name_to_index_.clear();
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_name_to_index_[params_.joints[i]] = i;
  }

  RCLCPP_INFO(get_logger(),
    "Initialized: equilibrium=[%.3f, %.3f, %.3f], control=100Hz",
    X_BP_desired.translation()(0), X_BP_desired.translation()(1), X_BP_desired.translation()(2));
}


// Constructor: init params, state vectors, ROS I/O, and default equilibrium pose
AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  RCLCPP_INFO_ONCE(get_logger(), "Initializing UR Admittance Controller - 6-DOF Force-Compliant Motion Control");

  InitializeParameters();
  InitializeStateVectors();
  SetupROSInterfaces();
  SetDefaultEquilibrium();
}


}  // namespace ur_admittance_controller


// Main entry: Constructor → initialize() → spin_some() + control_cycle() loop
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();

  // Tier 1: Node must be successfully created

  // Tier 2: Setup failures throw exceptions
  try {
    node->Initialize();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"),
                 "Failed to initialize admittance controller: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  rclcpp::Rate loop_rate(ur_admittance_controller::constants::CONTROL_LOOP_HZ);

  RCLCPP_INFO_ONCE(node->get_logger(), "Starting synchronized admittance control loop at 100Hz...");

  while (rclcpp::ok()) {
    executor.spin_some();

    node->ControlCycle();

    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Shutting down admittance controller...");
  rclcpp::shutdown();
  return 0;
}
