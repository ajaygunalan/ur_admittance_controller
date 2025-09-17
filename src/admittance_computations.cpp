
/**
 * @file admittance_computations.cpp
 * @brief Implementation of math helpers + AdmittanceNode core (pseudocode-accurate).
 *
 * Key choices vs old code:
 *  - Admittance no longer depends on X_WB_current. It uses only X_WB_des and the wrench.
 *  - Wrench is assumed to arrive in WORLD/base (per README wrench_node). It is re-expressed
*    to B_des with the force adjoint of X_BdesW.  (If your topic is in PROBE axes, convert
*    it in wrench_node; keeping that conversion outside preserves clarity.)
*  - Offset rates use FULL twist adjoint (adds p_hat_des * R_des * omega) — the missing physics term.
 */

#include "admittance_computations.hpp"

namespace ur_admittance_controller {

// -------- Adjoint builders ---------------------------------------------------

Matrix6d AdTwist(const Matrix3d& R, const Vector3d& p) {
  Matrix6d Ad = Matrix6d::Zero();
  Ad.block<3,3>(0,0) = R;
  Ad.block<3,3>(0,3) = Matrix3d::Zero();
  Ad.block<3,3>(3,0) = Skew(p) * R;
  Ad.block<3,3>(3,3) = R;
  return Ad;
}

Matrix6d AdForce(const Matrix3d& R, const Vector3d& p) {
  return AdTwist(R, p).transpose();
}

Adjoints BuildAdjointsFromDesired(const Matrix3d& R_des, const Vector3d& p_des) {
  Adjoints A;
  A.Ad_pose.setZero();
  A.Ad_pose.block<3,3>(0,0) = R_des;
  A.Ad_pose.block<3,3>(3,3) = R_des;
  A.Ad_twist = AdTwist(R_des, p_des);
  return A;
}

Vector6d ExpressWrenchWorldToBdes(const Vector6d& F_world,
                                  const Matrix3d& R_des,
                                  const Vector3d& p_des) {
  // Spatial wrench transform: F_B = Ad_{X_WB}^T F_W.
  // Set p_des = 0 if upstream processing already shifts the moment to the TCP origin.
  return AdForce(R_des, p_des) * F_world;
}

// -------- ODE helpers (unchanged math, clearer names) -----------------------

Vector6d ComputeAdmittanceAcceleration(
    const Vector6d& F_ext,
    const Vector6d& v_state,
    const Vector6d& x_state,
    const Vector6d& M_inv,
    const Vector6d& D,
    const Vector6d& K) {
  return M_inv.array() * (F_ext.array() - D.array()*v_state.array() - K.array()*x_state.array());
}

Vector6d LimitAcceleration(const Vector6d& a, double max_lin, double max_ang) {
  Vector6d out = a;
  const double ln = a.head<3>().norm();
  if (ln > max_lin && ln > 1e-12) out.head<3>() *= (max_lin/ln);
  const double an = a.tail<3>().norm();
  if (an > max_ang && an > 1e-12) out.tail<3>() *= (max_ang/an);
  return out;
}

Vector6d IntegrateVelocity(const Vector6d& v, const Vector6d& a, double dt) {
  return v + a * dt;
}

// -------- Kinematics --------------------------------------------------------
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

Result<Eigen::Isometry3d> ComputeForwardKinematics(const std::vector<double>& q_joints,
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
        fmt::format("FK failed: KDL error={}, input_joints={}, q_kdl_size={}", fk_result, q_joints.size(), q_kdl.data.size())));
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

Result<std::vector<double>> ComputeInverseKinematicsVelocity(
    const Vector6d& V_tool_W,
    const Eigen::Isometry3d& X_tool_W,
    const KDL::Frame& X_wrist_W,
    const std::vector<double>& q_current,
    KDL::ChainIkSolverVel_wdls* ik_solver) {

  KDL::JntArray q_kdl(q_current.size());
  for (size_t i = 0; i < q_current.size(); ++i) q_kdl(i) = q_current[i];

  // TCP (tool) twist in world
  KDL::Twist V_tool_kdl;
  V_tool_kdl.vel = KDL::Vector(V_tool_W(0), V_tool_W(1), V_tool_W(2));
  V_tool_kdl.rot = KDL::Vector(V_tool_W(3), V_tool_W(4), V_tool_W(5));

  // Shift twist from tool (TCP) to wrist point (world expression).
  KDL::Vector p_tool_wrist(
      X_tool_W.translation()(0) - X_wrist_W.p.x(),
      X_tool_W.translation()(1) - X_wrist_W.p.y(),
      X_tool_W.translation()(2) - X_wrist_W.p.z());

  KDL::Twist V_wrist_kdl;
  V_wrist_kdl.vel = V_tool_kdl.vel - V_tool_kdl.rot * p_tool_wrist;  // v - omega × p
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

// -------- Pose utilities ----------------------------------------------------

Vector6d ComputePoseError(const Eigen::Isometry3d& X_cmd_W,
                          const Eigen::Isometry3d& X_meas_W) {
  Vector6d error; // [pos; ori]
  error.head<3>() = X_cmd_W.translation() - X_meas_W.translation();

  Eigen::Quaterniond q_cmd(X_cmd_W.rotation());
  Eigen::Quaterniond q_meas(X_meas_W.rotation());
  if (q_cmd.dot(q_meas) < 0.0) q_meas.coeffs() *= -1.0;

  Eigen::Quaterniond q_err = (q_cmd * q_meas.inverse()).normalized();
  Eigen::AngleAxisd aa(q_err);
  error.tail<3>() = aa.axis() * aa.angle();
  return error;
}

std::pair<double,double> GetPoseErrorNorms(const Vector6d& e) {
  return {e.head<3>().norm(), e.tail<3>().norm()};
}

Vector6d ApplyWorkspaceLimits(const Vector6d& v,
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

Vector6d LimitVelocityMagnitude(const Vector6d& v,
                                double max_lin, double max_ang) {
  Vector6d out = v;
  double ln = v.head<3>().norm();
  if (ln > max_lin && ln > 1e-12) out.head<3>() *= (max_lin / ln);
  double an = v.tail<3>().norm();
  if (an > max_ang && an > 1e-12) out.tail<3>() *= (max_ang / an);
  return out;
}

// ================= AdmittanceNode member definitions =======================

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

void AdmittanceNode::ComputePoseError() {
  // e_X_W = (X_cmd ⊖ X_meas) : "cmd − meas"
  X_BP_error = ::ur_admittance_controller::ComputePoseError(g_X_WB_cmd, X_BP_current);

  auto [pos_err, ori_err] = ::ur_admittance_controller::GetPoseErrorNorms(X_BP_error);
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
    "Err(cmd−meas): pos=%.4fm [%.3f,%.3f,%.3f], ori=%.4frad [%.3f,%.3f,%.3f]",
    pos_err, X_BP_error(0), X_BP_error(1), X_BP_error(2),
    ori_err, X_BP_error(3), X_BP_error(4), X_BP_error(5));
}

/**
 * @brief Core admittance update (Steps 0–5). NO use of X_WB_current.
 *        Uses desired pose + incoming WORLD wrench, per README's wrench node.  fileciteturn0file5
 *        Math follows your pseudocode exactly (factored adjoints).            fileciteturn0file4
 */
void AdmittanceNode::ComputeAdmittance() {
  const double dt = control_period_.seconds();
  const Matrix3d R_des = X_BP_desired.rotation();
  const Vector3d p_des = X_BP_desired.translation();

  // --- Step 0: Re-express external wrench (WORLD -> B_des) using force adjoint.
  // NOTE: We assume /netft/proc_probe is already gravity-compensated & in WORLD/base.
  // If that's not the case, convert it in the wrench node (see README).          fileciteturn0file5
  Vector6d F_Bdes = ExpressWrenchWorldToBdes(F_P_B /*world*/, R_des, p_des);

  // Optional scaling knob
  F_Bdes *= admittance_ratio_;

  // --- Step 1: Admittance ODE in B_des
  Vector6d acc = ComputeAdmittanceAcceleration(
      F_Bdes, g_deltaXdot_Bdes, g_deltaX_Bdes, M_inverse_diag, D_diag, K_diag);

  acc = LimitAcceleration(acc, arm_max_acc_,
                          params_.admittance.limits.max_angular_acceleration);

  g_deltaXdot_Bdes = IntegrateVelocity(g_deltaXdot_Bdes, acc, dt);
  g_deltaX_Bdes    = IntegrateVelocity(g_deltaX_Bdes,    g_deltaXdot_Bdes, dt);

  // --- Step 2: Build adjoints from desired pose and map offset & rate to WORLD.
  const Adjoints A = BuildAdjointsFromDesired(R_des, p_des);

  const Vector6d dX_W    = A.Ad_pose  * g_deltaX_Bdes;
  const Vector6d dXdot_W = A.Ad_twist * g_deltaXdot_Bdes;

  const Vector3d dpos_W     = dX_W.head<3>();
  const Vector3d drot_W     = dX_W.tail<3>();
  const Vector3d dposdot_W  = dXdot_W.head<3>();
  const Vector3d drotdot_W  = dXdot_W.tail<3>();

  // --- Step 3: Compose commanded pose in WORLD.
  const double th = drot_W.norm();
  Matrix3d R_cmd = (th < 1e-12) ? R_des
                                : (Eigen::AngleAxisd(th, drot_W / th).toRotationMatrix() * R_des);

  g_X_WB_cmd = Eigen::Isometry3d::Identity();
  g_X_WB_cmd.linear()      = R_cmd;
  g_X_WB_cmd.translation() = p_des + dpos_W;

  // --- Step 4: Pose error will be computed after FK (cmd − meas) in ComputePoseError().

  // --- Step 5: World twist command: V_cmd = delta(Xdot)_W + K_v * e_W
  // e_W is computed after FK; store the open-loop term here and add K_v ∘ e_W later.
  Vector6d V_cmd_partial = Vector6d::Zero();
  V_cmd_partial.head<3>() = dposdot_W;
  V_cmd_partial.tail<3>() = drotdot_W;

  // Temporarily store partial; add P*error in ControlCycle after ComputePoseError.
  V_P_B_commanded = V_cmd_partial;
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

} // namespace ur_admittance_controller
