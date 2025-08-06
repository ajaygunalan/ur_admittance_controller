#include "admittance_computations.hpp"

namespace ur_admittance_controller {

// Kinematics helper
namespace kinematics {
    Result<KinematicsComponents> InitializeFromUrdf(
        const urdf::Model& urdf_model,
        const std::string& base_link,
        const std::string& tip_link) {

        KinematicsComponents components;

        // Convert URDF to KDL tree
        if (!kdl_parser::treeFromUrdfModel(urdf_model, components.tree)) {
            return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                           "Failed to convert URDF to KDL tree"));
        }

        // Extract chain from base to wrist (tool0 = wrist_3_link for UR robots)
        if (!components.tree.getChain(base_link, "wrist_3_link", components.robot_chain)) {
            return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                fmt::format("Failed to extract chain from {} to wrist_3_link", base_link)));
        }

        // Get transform from wrist to actual tip (through all fixed joints and sensor frames)
        KDL::Chain tool_chain;
        if (!components.tree.getChain("wrist_3_link", tip_link, tool_chain)) {
            // If direct chain fails, just use identity (wrist_3 = tool0)
            components.tool_offset = KDL::Frame::Identity();
        } else {
            // Compute full transform through all segments (including sensor frames)
            components.tool_offset = KDL::Frame::Identity();
            for (unsigned int i = 0; i < tool_chain.getNrOfSegments(); ++i) {
                components.tool_offset = components.tool_offset * tool_chain.getSegment(i).getFrameToTip();
            }
        }

        components.num_joints = components.robot_chain.getNrOfJoints();

        RCLCPP_DEBUG(rclcpp::get_logger("kinematics"),
                     "Initialized KDL: %zu joints, %d segments",
                     components.num_joints, components.robot_chain.getNrOfSegments());

        return components;
    }
}

inline Vector6d ComputeAdmittanceAcceleration(
    const Vector6d& external_wrench,
    const Vector6d& velocity_commanded,
    const Vector6d& pose_error,
    const Vector6d& mass_inverse,
    const Vector6d& damping,
    const Vector6d& stiffness) {
  return mass_inverse.array() *
      (external_wrench.array() - damping.array() * velocity_commanded.array() -
       stiffness.array() * pose_error.array());
}

inline Vector6d LimitAcceleration(
    const Vector6d& acceleration,
    double max_linear_acc,
    double max_angular_acc) {
  Vector6d limited = acceleration;
  double linear_norm = acceleration.head<3>().norm();
  if (linear_norm > max_linear_acc) {
    limited.head<3>() *= (max_linear_acc / linear_norm);
  }
  double angular_norm = acceleration.tail<3>().norm();
  if (angular_norm > max_angular_acc) {
    limited.tail<3>() *= (max_angular_acc / angular_norm);
  }
  return limited;
}

inline Vector6d IntegrateVelocity(
    const Vector6d& current_velocity,
    const Vector6d& acceleration,
    double dt) {
  return current_velocity + acceleration * dt;
}

inline Result<Eigen::Isometry3d> ComputeForwardKinematics(
    const std::vector<double>& q_joints,
    KDL::ChainFkSolverPos_recursive* fk_solver,
    const KDL::Frame& tool_offset) {

  KDL::JntArray q_kdl(q_joints.size());
  for (size_t i = 0; i < q_joints.size(); ++i) {
    q_kdl(i) = std::atan2(std::sin(q_joints[i]), std::cos(q_joints[i]));
  }

  KDL::Frame X_base_joint;
  int fk_result = fk_solver->JntToCart(q_kdl, X_base_joint);
  if (fk_result < 0) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
        fmt::format("FK failed: KDL error={}, input_joints={}, q_kdl_size={}",
                    fk_result, q_joints.size(), q_kdl.data.size())));
  }

  KDL::Frame X_base_tool = X_base_joint * tool_offset;

  Eigen::Isometry3d result;
  result.translation() = Eigen::Vector3d(X_base_tool.p.x(),
                                        X_base_tool.p.y(),
                                        X_base_tool.p.z());

  double x, y, z, w;
  X_base_tool.M.GetQuaternion(x, y, z, w);
  result.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();

  return result;
}

inline Result<std::vector<double>> ComputeInverseKinematicsVelocity(
    const Vector6d& V_tool,
    const Eigen::Isometry3d& X_tool,
    const KDL::Frame& X_wrist,
    const std::vector<double>& q_current,
    KDL::ChainIkSolverVel_wdls* ik_solver) {

  KDL::JntArray q_kdl(q_current.size());
  for (size_t i = 0; i < q_current.size(); ++i) {
    q_kdl(i) = q_current[i];
  }

  KDL::Twist V_tool_kdl;
  V_tool_kdl.vel = KDL::Vector(V_tool(0), V_tool(1), V_tool(2));
  V_tool_kdl.rot = KDL::Vector(V_tool(3), V_tool(4), V_tool(5));

  KDL::Vector p_tool_wrist = KDL::Vector(
    X_tool.translation()(0) - X_wrist.p.x(),
    X_tool.translation()(1) - X_wrist.p.y(),
    X_tool.translation()(2) - X_wrist.p.z()
  );

  KDL::Twist V_wrist_kdl;
  V_wrist_kdl.vel = V_tool_kdl.vel - V_tool_kdl.rot * p_tool_wrist;
  V_wrist_kdl.rot = V_tool_kdl.rot;

  KDL::JntArray v_kdl(q_current.size());
  if (ik_solver->CartToJnt(q_kdl, V_wrist_kdl, v_kdl) < 0) {
    return tl::unexpected(MakeError(ErrorCode::kIKSolverFailed,
                                   "Inverse kinematics velocity computation failed"));
  }

  std::vector<double> joint_velocities(q_current.size());
  for (size_t i = 0; i < q_current.size(); ++i) {
    if (std::isnan(v_kdl(i))) {
      return tl::unexpected(MakeError(ErrorCode::kIKSolverFailed,
                                     "IK returned NaN for joint " + std::to_string(i)));
    }
    joint_velocities[i] = v_kdl(i);
  }

  return joint_velocities;
}

inline Vector6d ComputePoseError(
    const Eigen::Isometry3d& X_current,
    const Eigen::Isometry3d& X_desired) {
  Vector6d error;
  error.head<3>() = X_current.translation() - X_desired.translation();

  Eigen::Quaterniond q_current(X_current.rotation());
  Eigen::Quaterniond q_desired(X_desired.rotation());

  if (q_current.dot(q_desired) < 0.0) {
    q_desired.coeffs() *= -1.0;  // Ensure shortest rotation path
  }

  Eigen::Quaterniond q_error = (q_current * q_desired.inverse()).normalized();
  Eigen::AngleAxisd aa_error(q_error);
  error.tail<3>() = aa_error.axis() * aa_error.angle();

  return error;
}

inline std::pair<double, double> GetPoseErrorNorms(const Vector6d& pose_error) {
  return {pose_error.head<3>().norm(), pose_error.tail<3>().norm()};
}

inline Vector6d ApplyWorkspaceLimits(
    const Vector6d& velocity,
    const Eigen::Vector3d& current_position,
    const Vector6d& workspace_limits) {
  Vector6d limited_velocity = velocity;
  for (size_t i = 0; i < 3; ++i) {
    const double min = workspace_limits[i * 2];
    const double max = workspace_limits[i * 2 + 1];
    if (current_position[i] <= min) {
      limited_velocity[i] = std::max(0.0, limited_velocity[i]);
    }
    if (current_position[i] >= max) {
      limited_velocity[i] = std::min(0.0, limited_velocity[i]);
    }
  }
  return limited_velocity;
}

inline Vector6d LimitVelocityMagnitude(
    const Vector6d& velocity,
    double max_linear_vel,
    double max_angular_vel) {
  Vector6d limited = velocity;
  double linear_norm = velocity.head<3>().norm();
  if (linear_norm > max_linear_vel) {
    limited.head<3>() *= (max_linear_vel / linear_norm);
  }
  double angular_norm = velocity.tail<3>().norm();
  if (angular_norm > max_angular_vel) {
    limited.tail<3>() *= (max_angular_vel / angular_norm);
  }
  return limited;
}




void AdmittanceNode::GetXBPCurrent() {
  RCLCPP_DEBUG_ONCE(get_logger(), "FK solver: %zu joints, %d segments",
                    num_joints_, kdl_chain_.getNrOfSegments());

  auto result = ComputeForwardKinematics(q_current_, fk_pos_solver_.get(), X_W3P);
  if (!result) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS, "FK failed: %s",
                          result.error().message.c_str());
    return;  // Keep using last valid pose
  }

  X_BP_current = result.value();

  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl_(i) = q_current_[i];
  }
  fk_pos_solver_->JntToCart(q_kdl_, X_BW3);
}

void AdmittanceNode::ComputePoseError() {
  X_BP_error = ::ur_admittance_controller::ComputePoseError(X_BP_current, X_BP_desired);

  auto [pos_err, ori_err] = ::ur_admittance_controller::GetPoseErrorNorms(X_BP_error);
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
    "Error: pos=%.4fm [%.3f,%.3f,%.3f], ori=%.4frad [%.3f,%.3f,%.3f]",
    pos_err, X_BP_error(0), X_BP_error(1), X_BP_error(2),
    ori_err, X_BP_error(3), X_BP_error(4), X_BP_error(5));
}


void AdmittanceNode::ComputeAdmittance() {
  Vector6d scaled_wrench = admittance_ratio_ * F_P_B;

  Vector6d acceleration = ComputeAdmittanceAcceleration(
      scaled_wrench, V_P_B_commanded, X_BP_error,
      M_inverse_diag, D_diag, K_diag);

  acceleration = LimitAcceleration(acceleration, arm_max_acc_,
                                              params_.admittance.limits.max_angular_acceleration);

  V_P_B_commanded = IntegrateVelocity(V_P_B_commanded, acceleration,
                                                 control_period_.seconds());

  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
    "Vel: [%.3f,%.3f,%.3f]m/s, Err: [%.3f,%.3f,%.3f]m",
    V_P_B_commanded(0), V_P_B_commanded(1), V_P_B_commanded(2),
    X_BP_error(0), X_BP_error(1), X_BP_error(2));
}

void AdmittanceNode::LimitToWorkspace() {
  const auto& pos = X_BP_current.translation();

  V_P_B_commanded = ApplyWorkspaceLimits(V_P_B_commanded, pos, workspace_limits_);
  V_P_B_commanded = LimitVelocityMagnitude(V_P_B_commanded, arm_max_vel_,
                                                       params_.admittance.limits.max_angular_velocity);
}

void AdmittanceNode::ComputeAndPubJointVelocities() {
  auto result = ComputeInverseKinematicsVelocity(
      V_P_B_commanded, X_BP_current, X_BW3, q_current_, ik_vel_solver_.get());

  if (!result) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
        "IK failed: vel=[%.3f,%.3f,%.3f]m/s - safety stop",
        V_P_B_commanded(0), V_P_B_commanded(1), V_P_B_commanded(2));
    std::fill(q_dot_cmd_.begin(), q_dot_cmd_.end(), 0.0);
  } else {
    q_dot_cmd_ = result.value();
  }

  velocity_msg_.data = q_dot_cmd_;
  velocity_pub_->publish(velocity_msg_);
}

Status AdmittanceNode::LoadKinematics() {
  RCLCPP_INFO(get_logger(), "Loading robot kinematics from URDF");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      this, "/robot_state_publisher");

  if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                   "Robot state publisher service not available"));
  }

  auto parameters = parameters_client->get_parameters({"robot_description"});
  if (parameters.empty() || parameters[0].get_type() == rclcpp::PARAMETER_NOT_SET) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                   "robot_description parameter not set"));
  }

  urdf::Model urdf_model;
  if (!urdf_model.initString(parameters[0].as_string())) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                   "Failed to parse URDF model"));
  }

  auto result = kinematics::InitializeFromUrdf(urdf_model, params_.base_link, params_.tip_link);
  if (!result) {
    RCLCPP_ERROR(get_logger(), "%s", result.error().message.c_str());
    return tl::unexpected(result.error());
  }

  auto& components = result.value();
  kdl_tree_ = std::move(components.tree);
  kdl_chain_ = components.robot_chain;
  X_W3P = components.tool_offset;

  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_, 1e-5, 150);
  ik_vel_solver_->setLambda(0.1);

  num_joints_ = kdl_chain_.getNrOfJoints();

  q_kdl_.resize(num_joints_);
  v_kdl_.resize(num_joints_);

  RCLCPP_INFO(get_logger(), "Kinematics ready: %zu joints, %s->%s",
              num_joints_, params_.base_link.c_str(), params_.tip_link.c_str());
  logging::LogVector3(get_logger(), "  Tool offset:", Vector3d(X_W3P.p.x(), X_W3P.p.y(), X_W3P.p.z()));
  return {};
}


}  // namespace ur_admittance_controller
