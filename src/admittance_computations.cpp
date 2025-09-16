// admittance_computations.cpp — v3-fixed admittance flow (pseudocode-accurate)
// - ODE on offset in B_des
// - Pose-offset mapping: diag(R_des, R_des)  [no coupling]
// - Offset-rate mapping: full Ad_twist with  p̂ R  coupling  [this fixes the bug]
// - Commanded pose composed explicitly; error = cmd − meas (world)
// - KDL vel-IK and wrist shift unchanged

#include "admittance_computations.hpp"

namespace ur_admittance_controller {

// --------- File-scope state for the offset ODE & commanded pose (no header changes) ---------
namespace {
  // Internal ordering is [linear; angular] to match KDL::Twist (vel first, then rot).
  Vector6d g_deltaX_Bdes    = Vector6d::Zero();  // [δp; δr] in B_des axes
  Vector6d g_deltaXdot_Bdes = Vector6d::Zero();  // [δṗ; δṙ] in B_des axes
  Eigen::Isometry3d g_X_BP_cmd = Eigen::Isometry3d::Identity();  // commanded pose in world
}

// ------------- Kinematics helper (unchanged) ------------------------------------------------
namespace kinematics {
Result<KinematicsComponents> InitializeFromUrdf(const urdf::Model& urdf_model,
                                                const std::string& base_link,
                                                const std::string& tip_link) {
  KinematicsComponents components;

  if (!kdl_parser::treeFromUrdfModel(urdf_model, components.tree)) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                    "Failed to convert URDF to KDL tree"));
  }

  if (!components.tree.getChain(base_link, "wrist_3_link", components.robot_chain)) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
        fmt::format("Failed to extract chain from {} to wrist_3_link", base_link)));
  }

  KDL::Chain tool_chain;
  if (!components.tree.getChain("wrist_3_link", tip_link, tool_chain)) {
    components.tool_offset = KDL::Frame::Identity();
  } else {
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
}  // namespace kinematics

// ------------- Small helpers (unchanged) ----------------------------------------------------
inline Vector6d ComputeAdmittanceAcceleration(const Vector6d& external_wrench,
                                              const Vector6d& vel_state,
                                              const Vector6d& pos_state,
                                              const Vector6d& mass_inverse,
                                              const Vector6d& damping,
                                              const Vector6d& stiffness) {
  // Element-wise: M^{-1} (F - D * v - K * x)
  return mass_inverse.array() *
         (external_wrench.array()
          - damping.array()   * vel_state.array()
          - stiffness.array() * pos_state.array());
}

inline Vector6d LimitAcceleration(const Vector6d& a, double max_lin, double max_ang) {
  Vector6d limited = a;
  double ln = a.head<3>().norm();
  if (ln > max_lin && ln > 1e-12) limited.head<3>() *= (max_lin / ln);
  double an = a.tail<3>().norm();
  if (an > max_ang && an > 1e-12) limited.tail<3>() *= (max_ang / an);
  return limited;
}

inline Vector6d IntegrateVelocity(const Vector6d& v, const Vector6d& a, double dt) {
  return v + a * dt;
}

inline Result<Eigen::Isometry3d> ComputeForwardKinematics(const std::vector<double>& q_joints,
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

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(X_base_tool.p.x(),
                                         X_base_tool.p.y(),
                                         X_base_tool.p.z());
  double x, y, z, w;
  X_base_tool.M.GetQuaternion(x, y, z, w);
  result.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
  return result;
}

inline Result<std::vector<double>> ComputeInverseKinematicsVelocity(
    const Vector6d& V_tool, const Eigen::Isometry3d& X_tool,
    const KDL::Frame& X_wrist, const std::vector<double>& q_current,
    KDL::ChainIkSolverVel_wdls* ik_solver) {

  KDL::JntArray q_kdl(q_current.size());
  for (size_t i = 0; i < q_current.size(); ++i) q_kdl(i) = q_current[i];

  KDL::Twist V_tool_kdl;
  V_tool_kdl.vel = KDL::Vector(V_tool(0), V_tool(1), V_tool(2));
  V_tool_kdl.rot = KDL::Vector(V_tool(3), V_tool(4), V_tool(5));

  // tool (TCP) -> wrist point shift
  KDL::Vector p_tool_wrist = KDL::Vector(
      X_tool.translation()(0) - X_wrist.p.x(),
      X_tool.translation()(1) - X_wrist.p.y(),
      X_tool.translation()(2) - X_wrist.p.z());

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

inline Vector6d ComputePoseError(const Eigen::Isometry3d& X_current,
                                 const Eigen::Isometry3d& X_desired) {
  Vector6d error; // [pos; ori] (translation first)
  error.head<3>() = X_current.translation() - X_desired.translation();

  Eigen::Quaterniond q_cur(X_current.rotation());
  Eigen::Quaterniond q_des(X_desired.rotation());
  if (q_cur.dot(q_des) < 0.0) q_des.coeffs() *= -1.0;

  Eigen::Quaterniond q_err = (q_cur * q_des.inverse()).normalized();
  Eigen::AngleAxisd aa(q_err);
  error.tail<3>() = aa.axis() * aa.angle();
  return error;
}

inline std::pair<double,double> GetPoseErrorNorms(const Vector6d& e) {
  return {e.head<3>().norm(), e.tail<3>().norm()};
}

inline Vector6d ApplyWorkspaceLimits(const Vector6d& v,
                                     const Eigen::Vector3d& pos,
                                     const Vector6d& limits) {
  Vector6d out = v;
  for (size_t i = 0; i < 3; ++i) {
    const double min = limits[i*2], max = limits[i*2+1];
    if (pos[i] <= min) out[i] = std::max(0.0, out[i]);
    if (pos[i] >= max) out[i] = std::min(0.0, out[i]);
  }
  return out;
}

inline Vector6d LimitVelocityMagnitude(const Vector6d& v,
                                       double max_lin, double max_ang) {
  Vector6d out = v;
  double ln = v.head<3>().norm();
  if (ln > max_lin && ln > 1e-12) out.head<3>() *= (max_lin / ln);
  double an = v.tail<3>().norm();
  if (an > max_ang && an > 1e-12) out.tail<3>() *= (max_ang / an);
  return out;
}

// ------------- FK to get X_BP_current (unchanged) -------------------------------------------
void AdmittanceNode::GetXBPCurrent() {
  RCLCPP_DEBUG_ONCE(get_logger(), "FK solver: %zu joints, %d segments",
                    num_joints_, kdl_chain_.getNrOfSegments());

  auto result = ComputeForwardKinematics(q_current_, fk_pos_solver_.get(), X_W3P);
  if (!result) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
                          "FK failed: %s", result.error().message.c_str());
    return;  // keep last valid pose
  }

  X_BP_current = result.value();

  for (size_t i = 0; i < num_joints_; ++i) q_kdl_(i) = q_current_[i];
  fk_pos_solver_->JntToCart(q_kdl_, X_BW3);
}

// ------------- Pose error logging (cmd − meas, world) ---------------------------------------
void AdmittanceNode::ComputePoseError() {
  // e_X_W = (X_cmd ⊖ X_meas) : "cmd − meas"
  X_BP_error = ::ur_admittance_controller::ComputePoseError(g_X_BP_cmd, X_BP_current);

  auto [pos_err, ori_err] = ::ur_admittance_controller::GetPoseErrorNorms(X_BP_error);
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
    "Err(cmd−meas): pos=%.4fm [%.3f,%.3f,%.3f], ori=%.4frad [%.3f,%.3f,%.3f]",
    pos_err, X_BP_error(0), X_BP_error(1), X_BP_error(2),
    ori_err, X_BP_error(3), X_BP_error(4), X_BP_error(5));
}

// ------------- v3-fixed Admittance (core update) --------------------------------------------
void AdmittanceNode::ComputeAdmittance() {
  const double dt = control_period_.seconds();
  const Matrix3d R_des = X_BP_desired.rotation();
  const Vector3d p_des = X_BP_desired.translation();

  // (Step 0) Re-express external wrench from PROBE axes to B_des axes (dual adjoint)
  // X_BdesP = X_WB_des^{-1} * X_WP_current   (we assume PROBE=P; wrench is at TCP)
  Eigen::Isometry3d X_BdesP = X_BP_desired.inverse() * X_BP_current;
  const Matrix3d R = X_BdesP.rotation();
  const Vector3d p = X_BdesP.translation();

  Vector6d F_P_des; // [force; torque] in B_des axes
  {
    const Vector3d Fp = F_P_B.head<3>();   // incoming wrench is in PROBE axes
    const Vector3d Mp = F_P_B.tail<3>();
    const Vector3d Fd = R * Fp;
    const Vector3d Md = R * Mp + p.cross(Fd);
    F_P_des.head<3>() = Fd;
    F_P_des.tail<3>() = Md;
  }

  // Optional scaling of external wrench
  F_P_des *= admittance_ratio_;

  // (Step 1) Admittance ODE in B_des axes on the offset state
  Vector6d acc = ComputeAdmittanceAcceleration(
      F_P_des, g_deltaXdot_Bdes, g_deltaX_Bdes,
      M_inverse_diag, D_diag, K_diag);

  acc = LimitAcceleration(acc, arm_max_acc_,
                          params_.admittance.limits.max_angular_acceleration);

  g_deltaXdot_Bdes = IntegrateVelocity(g_deltaXdot_Bdes, acc, dt);
  g_deltaX_Bdes    = IntegrateVelocity(g_deltaX_Bdes,    g_deltaXdot_Bdes, dt);

  // (Step 2) Convert offset & rate to world using factored adjoints
  // Offsets: pose-adjoint (no coupling) — diag(R_des, R_des)
  Vector3d dpos_W = R_des * g_deltaX_Bdes.head<3>();
  Vector3d drot_W = R_des * g_deltaX_Bdes.tail<3>();

  // Rates: full twist-adjoint (with coupling): [v_W; ω_W] = [[R, p̂ R],[0,R]] [v_B; ω_B]
  Vector3d drotdot_W = R_des * g_deltaXdot_Bdes.tail<3>();                        // ω_W
  Vector3d dposdot_W = R_des * g_deltaXdot_Bdes.head<3>() + p_des.cross(drotdot_W); // v_W

  // (Step 3) Compose commanded pose in world
  const double th = drot_W.norm();
  Matrix3d R_cmd = (th < 1e-12) ? R_des
                                : (Eigen::AngleAxisd(th, drot_W / th).toRotationMatrix() * R_des);

  g_X_BP_cmd = Eigen::Isometry3d::Identity();
  g_X_BP_cmd.linear()      = R_cmd;
  g_X_BP_cmd.translation() = X_BP_desired.translation() + dpos_W;

  // (Step 4) Pose error in world (cmd − meas)
  Vector6d e_W = ::ur_admittance_controller::ComputePoseError(g_X_BP_cmd, X_BP_current);

  // (Step 5) World twist command: V_cmd = δẊ_W + Kp^W * e_W
  // Reuse K_diag as world P gain (matches current parameter set; can be split later)
  V_P_B_commanded.head<3>() = dposdot_W + K_diag.head<3>().cwiseProduct(e_W.head<3>());
  V_P_B_commanded.tail<3>() = drotdot_W + K_diag.tail<3>().cwiseProduct(e_W.tail<3>());

  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
    "V_cmd: lin=[%.3f,%.3f,%.3f] ang=[%.3f,%.3f,%.3f]",
    V_P_B_commanded(0), V_P_B_commanded(1), V_P_B_commanded(2),
    V_P_B_commanded(3), V_P_B_commanded(4), V_P_B_commanded(5));
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
        "IK failed: lin=[%.3f,%.3f,%.3f]m/s",
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
  kdl_tree_  = std::move(components.tree);
  kdl_chain_ = components.robot_chain;
  X_W3P      = components.tool_offset;

  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_, 1e-5, 150);
  ik_vel_solver_->setLambda(0.1);

  num_joints_ = kdl_chain_.getNrOfJoints();
  q_kdl_.resize(num_joints_);
  v_kdl_.resize(num_joints_);

  RCLCPP_INFO(get_logger(), "Kinematics ready: %zu joints, %s->%s",
              num_joints_, params_.base_link.c_str(), params_.tip_link.c_str());
  logging::LogVector3(get_logger(), "  Tool offset:",
                      Vector3d(X_W3P.p.x(), X_W3P.p.y(), X_W3P.p.z()));
  return {};
}

} 