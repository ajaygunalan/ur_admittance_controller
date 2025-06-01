// UR Admittance Controller Node Definition
#ifndef UR_ADMITTANCE_NODE_HPP
#define UR_ADMITTANCE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

// Kinematics
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <urdf/model.h>

// Reuse existing parameter system
#include "ur_admittance_controller/ur_admittance_controller_parameters.hpp"

// Reuse existing types
#include "admittance_node_types.hpp"
#include "admittance_constants.hpp"

// Standard includes
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <mutex>
#include <vector>
#include <thread>

namespace ur_admittance_controller {

// Thread-safe data structure for node-thread communication
struct ComputedControl {
    std::vector<double> joint_velocities;
    rclcpp::Time computation_timestamp;
    bool valid = false;
    
    ComputedControl() : valid(false) {}
    ComputedControl(size_t joint_count) 
        : joint_velocities(joint_count, 0.0), valid(false) {}
};

class AdmittanceNode : public rclcpp::Node
{
public:
  explicit AdmittanceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AdmittanceNode();

private:
  // Core control loop
  void controlLoop();
  
  // Callback functions
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void robotDescriptionCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  
  
  // Initialize components
  bool loadKinematics();
  bool initializeDesiredPose();  // Set desired pose to current robot pose
  
  // Core algorithm functions (from admittance_computations.cpp)
  bool computeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out);
  Vector6d computePoseError_tip_base();
  void updateMassMatrix();
  void updateDampingMatrix();
  
  // Parameter callback system
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters);
  bool convertToJointSpace(const Vector6d& cartesian_velocity, const rclcpp::Duration& period);
  bool handleDriftReset();
  // Helper functions
  bool checkDeadband();
  
  // New node computation methods
  void computationTimerCallback();
  bool computeAdmittanceControlInNode(const rclcpp::Duration& period, 
                                     std::vector<double>& joint_velocities_out);
  void integrateAndPublish();
  bool validatePoseErrorSafety(const Vector6d& pose_error);
  
  // Direct transform functions (replacing cache system)
  Vector6d transformWrench(const Vector6d& wrench_sensor_frame);
  bool getCurrentEndEffectorPose(Eigen::Isometry3d& pose);
  
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  
  // Publishers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  
  
  
  // Thread-based control loop
  std::thread control_thread_;
  std::atomic<bool> running_{false};
  void controlThreadFunction();
  
  // Node-thread communication
  ComputedControl computed_control_;
  std::mutex control_data_mutex_;
  
  // Computation timer (replaces computation in thread)
  rclcpp::TimerBase::SharedPtr computation_timer_;
  
  // Parameters (reuse existing param structure)
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;
  
  // Transform handling
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // State variables
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> current_pos_;
  geometry_msgs::msg::WrenchStamped current_wrench_;
  std::mutex wrench_mutex_;
  std::mutex joint_state_mutex_;
  
  // Robot description data
  std::string robot_description_;
  std::mutex robot_description_mutex_;
  std::atomic<bool> robot_description_received_{false};
  
  // Desired pose initialization
  std::atomic<bool> desired_pose_initialized_{false};
  std::mutex desired_pose_mutex_;
  
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  
  // Control variables (to be ported from existing controller)
  Vector6d F_sensor_base_;
  Vector6d V_base_tip_base_;
  Vector6d desired_vel_;
  Vector6d desired_accel_;
  
  // Admittance parameters
  Matrix6d mass_;
  Matrix6d mass_inverse_;
  Matrix6d damping_;
  Matrix6d stiffness_;
  
  // Transform caches
  Eigen::Isometry3d X_base_tip_current_;
  Eigen::Isometry3d X_base_tip_desired_;
  
  // Additional algorithm variables
  Vector6d error_tip_base_;
  Vector6d wrench_filtered_;
  
  
  // Pre-allocated messages for performance
  trajectory_msgs::msg::JointTrajectory trajectory_msg_;
  
  
  // Direct KDL kinematics
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
  bool kinematics_ready_ = false;
  
  
  
  // Drift reset (unused variables removed)
};

}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_NODE_HPP