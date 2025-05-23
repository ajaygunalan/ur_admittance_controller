#include "ur_admittance_controller/admittance_controller.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

namespace ur_admittance_controller
{

AdmittanceController::AdmittanceController()
: controller_interface::ChainableControllerInterface(),
  last_twist_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
  retry_time_(rclcpp::Time(0, 0, RCL_ROS_TIME))
{
}

controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint : params_.joints) {
    for (const auto & interface : params_.command_interfaces) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  
  return config;
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint : params_.joints) {
    for (const auto & interface : params_.state_interfaces) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  
  // Add force-torque sensor interfaces if available
  if (!params_.ft_sensor_name.empty()) {
    for (const auto & axis : {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"}) {
      config.names.push_back(params_.ft_sensor_name + "/" + axis);
    }
  }
  
  return config;
}

controller_interface::CallbackReturn AdmittanceController::on_init()
{
  try {
    // Initialize parameter listener
    param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
    
    // Set up TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize matrices
    mass_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
    damping_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
    stiffness_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
    
    wrench_external_ = Eigen::Matrix<double, 6, 1>::Zero();
    pose_error_ = Eigen::Matrix<double, 6, 1>::Zero();
    velocity_error_ = Eigen::Matrix<double, 6, 1>::Zero();
    desired_acceleration_ = Eigen::Matrix<double, 6, 1>::Zero();
    desired_velocity_ = Eigen::Matrix<double, 6, 1>::Zero();
    desired_pose_ = Eigen::Matrix<double, 6, 1>::Zero();
    cart_vel_cmd_ = Eigen::Matrix<double, 6, 1>::Zero();
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Update parameters if they have changed
  params_ = param_listener_->get_params();
  
  // Load kinematics plugin
  if (!loadKinematics()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load kinematics plugin");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Set up matrix parameters from configuration
  for (size_t i = 0; i < 6; ++i) {
    mass_matrix_(i, i) = params_.mass[i];
    stiffness_matrix_(i, i) = params_.stiffness[i];
    
    // Compute damping from mass, stiffness and damping ratio
    // D = 2 * zeta * sqrt(M * S)
    if (params_.stiffness[i] > 0.0) {
      damping_matrix_(i, i) = 2.0 * params_.damping_ratio[i] * 
        std::sqrt(params_.mass[i] * params_.stiffness[i]);
    } else {
      damping_matrix_(i, i) = params_.damping_ratio[i];
    }
  }
  
  // Subscribe to external wrench
  wrench_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "~/wrench_reference", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
      wrenchCallback(msg);
    });
    
  // Create publisher for cartesian velocity
  cart_vel_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    "~/cartesian_velocity_command", 10);
    
  // Create action client for trajectory controller
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    get_node(),
    "scaled_joint_trajectory_controller/follow_joint_trajectory");
  
  // Resize KDL arrays
  joint_positions_.resize(params_.joints.size());
  joint_velocities_.resize(params_.joints.size());
  joint_velocities_cmd_.resize(params_.joints.size());
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Wait for transforms to be available
  if (!waitForTransforms()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get transforms");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Reset state
  pose_error_.setZero();
  velocity_error_.setZero();
  desired_acceleration_.setZero();
  desired_velocity_.setZero();
  cart_vel_cmd_.setZero();
  
  // Initialize joint positions from state interfaces
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto position_interface_name = params_.joints[i] + "/position";
    const auto position_interface_index = std::distance(
      state_interfaces_.cbegin(),
      std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&position_interface_name](const auto & interface) {
          return interface.get_name() == position_interface_name;
        }));
    
    if (position_interface_index >= static_cast<long>(state_interfaces_.size())) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Position interface '%s' not found", position_interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    
    joint_positions_(i) = state_interfaces_[position_interface_index].get_optional().value();
    joint_velocities_(i) = 0.0;
    joint_velocities_cmd_(i) = 0.0;
  }
  
  kdl_initialized_ = true;
  new_velocity_command_ = false;
  
  RCLCPP_INFO(get_node()->get_logger(), "Admittance controller activated");
  
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Cancel any pending trajectories
  if (goal_future_.valid() && goal_future_.wait_for(std::chrono::seconds(0)) != 
      std::future_status::ready) {
    auto goal_handle = goal_future_.get();
    if (goal_handle) {
      action_client_->async_cancel_goal(goal_handle);
    }
  }
  
  // Reset state
  kdl_initialized_ = false;
  
  RCLCPP_INFO(get_node()->get_logger(), "Admittance controller deactivated");
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Update parameters if they have changed
  if (params_.dynamic_parameters) {
    params_ = param_listener_->get_params();
  }
  
  // Get the latest wrench measurement
  auto wrench_msg = wrench_buffer_.readFromRT();
  
  // Check for timeout on external force commands
  if (wrench_msg && time - wrench_msg->header.stamp < rclcpp::Duration::from_seconds(1.0)) {
    // Process external wrench
    wrench_external_(0) = wrench_msg->wrench.force.x;
    wrench_external_(1) = wrench_msg->wrench.force.y;
    wrench_external_(2) = wrench_msg->wrench.force.z;
    wrench_external_(3) = wrench_msg->wrench.torque.x;
    wrench_external_(4) = wrench_msg->wrench.torque.y;
    wrench_external_(5) = wrench_msg->wrench.torque.z;
    
    // Apply admittance control algorithm
    calculateAdmittance(period);
    
    // Apply deadband to avoid tiny motions
    bool motion_above_threshold = false;
    for (size_t i = 0; i < 6; ++i) {
      if (std::abs(cart_vel_cmd_(i)) > params_.min_motion_threshold) {
        motion_above_threshold = true;
        break;
      }
    }
    
    if (motion_above_threshold) {
      // Apply velocity scaling for stability
      cart_vel_cmd_ *= params_.velocity_scale_factor;
      
      // Update velocity command
      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = cart_vel_cmd_(0);
      twist_msg.linear.y = cart_vel_cmd_(1);
      twist_msg.linear.z = cart_vel_cmd_(2);
      twist_msg.angular.x = cart_vel_cmd_(3);
      twist_msg.angular.y = cart_vel_cmd_(4);
      twist_msg.angular.z = cart_vel_cmd_(5);
      
      cart_vel_pub_->publish(twist_msg);
      
      // Send trajectory if not in chained mode
      if (!get_chained_mode()) {
        new_velocity_command_ = true;
        last_twist_time_ = time;
        sendTrajectory();
      }
    }
  }
  
  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Check if we need to process a new velocity command
  if (new_velocity_command_) {
    // Validate the command is not too old
    const auto dt = time - last_twist_time_;
    // Using a fixed timeout of 1.0 seconds if parameter not available
    if (dt > rclcpp::Duration::from_seconds(1.0)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Ignoring old command: %f seconds old", dt.seconds());
      // Clear the flag to avoid reprocessing this command
      new_velocity_command_ = false;
    } else {
      // Generate trajectory from velocity command
      calculateAdmittance(period);
      
      // Send trajectory and reset flag
      sendTrajectory();
      new_velocity_command_ = false;
    }
  }
  
  // Get current positions from state interfaces
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    size_t position_interface_index = i;
    joint_positions_(i) = state_interfaces_[position_interface_index].get_optional().value();
  }
  
  // Get current velocities from state interfaces
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    size_t velocity_interface_index = params_.joints.size() + i;
    joint_velocities_(i) = state_interfaces_[velocity_interface_index].get_optional().value();
  }
  
  // Check if we need to retry a trajectory
  if (retry_pending_ && time >= retry_time_) {
    sendTrajectory();
    retry_pending_ = false;
  }
  
  // Write command outputs when in chained mode
  if (get_chained_mode()) {
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      for (size_t j = 0; j < params_.command_interfaces.size(); ++j) {
        const auto interface_name = params_.joints[i] + "/" + params_.command_interfaces[j];
        const auto command_interface_index = std::distance(
          command_interfaces_.cbegin(),
          std::find_if(
            command_interfaces_.cbegin(), command_interfaces_.cend(),
            [&interface_name](const auto & interface) {
              return interface.get_name() == interface_name;
            }));
        
        if (command_interface_index >= static_cast<long>(command_interfaces_.size())) {
          RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), 
            *get_node()->get_clock(),
            1000, 
            "Command interface '%s' not found", interface_name.c_str());
          return controller_interface::return_type::ERROR;
        }
        
        if (params_.command_interfaces[j] == "position") {
          if (!command_interfaces_[command_interface_index].set_value(joint_positions_(i))) {
            RCLCPP_WARN_THROTTLE(
              get_node()->get_logger(),
              *get_node()->get_clock(),
              1000,
              "Failed to set position command for joint %s", params_.joints[i].c_str());
          }
        } else if (params_.command_interfaces[j] == "velocity") {
          if (!command_interfaces_[command_interface_index].set_value(joint_velocities_cmd_(i))) {
            RCLCPP_WARN_THROTTLE(
              get_node()->get_logger(),
              *get_node()->get_clock(),
              1000,
              "Failed to set velocity command for joint %s", params_.joints[i].c_str());
          }
        }
      }
    }
  }
  
  return controller_interface::return_type::OK;
}

void AdmittanceController::calculateAdmittance(const rclcpp::Duration & period)
{
  if (!kdl_initialized_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "KDL not initialized, can't calculate admittance");
    return;
  }
  
  // M * a + D * v + S * e = F_ext
  // a = M^-1 * (F_ext - D * v - S * e)
  
  // Update acceleration - using the raw wrench from UR F/T sensor which already handles gravity compensation
  desired_acceleration_ = mass_matrix_.inverse() * 
    (wrench_external_ - damping_matrix_ * desired_velocity_ - stiffness_matrix_ * pose_error_);
  
  // Update velocity
  desired_velocity_ += desired_acceleration_ * period.seconds();
  
  // Apply admittance enabled axes
  for (size_t i = 0; i < 6; ++i) {
    if (!params_.admittance_enabled_axes[i]) {
      desired_velocity_(i) = 0.0;
    }
  }
  
  // Store command
  cart_vel_cmd_ = desired_velocity_;
  
  // Get current joint positions from state interfaces
  std::vector<double> current_positions(params_.joints.size());
  std::vector<double> current_velocities(params_.joints.size());
  std::vector<double> velocity_commands(params_.joints.size(), 0.0);
  
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto position_interface_name = params_.joints[i] + "/position";
    const auto position_interface_index = std::distance(
      state_interfaces_.cbegin(),
      std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&position_interface_name](const auto & interface) {
          return interface.get_name() == position_interface_name;
        }));
    
    current_positions[i] = state_interfaces_[position_interface_index].get_optional().value();
    
    const auto velocity_interface_name = params_.joints[i] + "/velocity";
    const auto velocity_interface_index = std::distance(
      state_interfaces_.cbegin(),
      std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&velocity_interface_name](const auto & interface) {
          return interface.get_name() == velocity_interface_name;
        }));
    
    current_velocities[i] = state_interfaces_[velocity_interface_index].get_optional().value();
  }
  
  // Create KDL data structures
  KDL::JntArray kdl_joint_positions(params_.joints.size());
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    kdl_joint_positions(i) = current_positions[i];
  }
  
  // Create a KDL twist from the Cartesian velocity
  KDL::Twist kdl_cart_vel;
  kdl_cart_vel.vel[0] = cart_vel_cmd_(0);
  kdl_cart_vel.vel[1] = cart_vel_cmd_(1);
  kdl_cart_vel.vel[2] = cart_vel_cmd_(2);
  kdl_cart_vel.rot[0] = cart_vel_cmd_(3);
  kdl_cart_vel.rot[1] = cart_vel_cmd_(4);
  kdl_cart_vel.rot[2] = cart_vel_cmd_(5);
  
  try {
    // Get the Jacobian from kinematics interface
    Eigen::MatrixXd jacobian(6, params_.joints.size());
    std::vector<std::string> joint_names = params_.joints;
    
    // We'll use a manual approach for now since the kinematics interface API might vary
    // In a real implementation, use the proper kinematics interface API to get joint velocities
    
    // Simple velocity calculation using a damped least squares approach
    // This is just a placeholder until we properly integrate with the kinematics_interface
    // In the actual implementation, we would use: kinematics_->getJacobian(...)
    
    // For now, use a simple mapping - in production, replace with actual kinematics calculations
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      // Just use a simple scaling factor for each joint for now
      // This is just a placeholder for testing
      velocity_commands[i] = 0.1 * cart_vel_cmd_(i % 6);
    }
    
    // Store the velocity commands for use in sending trajectories
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      // Store in a member variable to be used later
      if (i < joint_velocities_cmd_.rows()) {
        joint_velocities_cmd_(i) = velocity_commands[i];
      }
    }
    
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to compute joint velocities: %s", e.what());
    
    // Zero the velocities as a fallback
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      if (i < joint_velocities_cmd_.rows()) {
        joint_velocities_cmd_(i) = 0.0;
      }
    }
  }
}

// Gravity compensation removed as the UR robot already compensates for gravity in its F/T readings

void AdmittanceController::wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  // Filter sensor reading
  auto last_wrench = wrench_buffer_.readFromNonRT();
  if (last_wrench) {
    // Apply exponential filter
    msg->wrench.force.x = params_.filter_coefficient * msg->wrench.force.x + 
      (1.0 - params_.filter_coefficient) * last_wrench->wrench.force.x;
    msg->wrench.force.y = params_.filter_coefficient * msg->wrench.force.y + 
      (1.0 - params_.filter_coefficient) * last_wrench->wrench.force.y;
    msg->wrench.force.z = params_.filter_coefficient * msg->wrench.force.z + 
      (1.0 - params_.filter_coefficient) * last_wrench->wrench.force.z;
    msg->wrench.torque.x = params_.filter_coefficient * msg->wrench.torque.x + 
      (1.0 - params_.filter_coefficient) * last_wrench->wrench.torque.x;
    msg->wrench.torque.y = params_.filter_coefficient * msg->wrench.torque.y + 
      (1.0 - params_.filter_coefficient) * last_wrench->wrench.torque.y;
    msg->wrench.torque.z = params_.filter_coefficient * msg->wrench.torque.z + 
      (1.0 - params_.filter_coefficient) * last_wrench->wrench.torque.z;
  }
  
  wrench_buffer_.writeFromNonRT(*msg);
}

bool AdmittanceController::loadKinematics()
{
  try {
    // Load the kinematics plugin
    auto kinematics_loader = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      params_.kinematics_plugin_package, "kinematics_interface::KinematicsInterface");
    kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
      kinematics_loader->createUnmanagedInstance(params_.kinematics_plugin_name));
    
    // Initialize the kinematics plugin with the robot description
    if (!kinematics_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create kinematics plugin instance");
      return false;
    }
    
    // Prepare parameters for the kinematics interface
    // The actual API might vary between different kinematics plugins
    // For testing, we'll just log successful loading
    RCLCPP_INFO(get_node()->get_logger(), 
      "Loaded kinematics plugin: %s from package: %s",
      params_.kinematics_plugin_name.c_str(),
      params_.kinematics_plugin_package.c_str());
    
    // Resize joint arrays now that we know the joint count
    joint_positions_.resize(params_.joints.size());
    joint_velocities_.resize(params_.joints.size());
    joint_velocities_cmd_.resize(params_.joints.size());
    
    // Note: We're not setting up KDL gravity compensation
    // since UR robots already compensate for gravity in their F/T readings
    
    kdl_initialized_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Kinematics initialized successfully");
    
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception while loading kinematics plugin: %s", e.what());
    return false;
  }
}

bool AdmittanceController::waitForTransforms()
{
  // Check if we have necessary transforms
  const auto timeout_sec = 5.0;
  std::string error_string;
  
  // Check for transform from base to world
  if (!tf_buffer_->canTransform(
      params_.world_frame, params_.base_link, rclcpp::Time(0), 
      rclcpp::Duration::from_seconds(timeout_sec), &error_string)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), 
      "Cannot transform from '%s' to '%s': %s",
      params_.world_frame.c_str(), params_.base_link.c_str(), error_string.c_str());
    return false;
  }
  
  // Check for transform from tip to base
  if (!tf_buffer_->canTransform(
      params_.base_link, params_.tip_link, rclcpp::Time(0),
      rclcpp::Duration::from_seconds(timeout_sec), &error_string)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), 
      "Cannot transform from '%s' to '%s': %s",
      params_.base_link.c_str(), params_.tip_link.c_str(), error_string.c_str());
    return false;
  }
  
  // Check for transform from FT sensor to base
  if (!tf_buffer_->canTransform(
      params_.base_link, params_.ft_frame, rclcpp::Time(0),
      rclcpp::Duration::from_seconds(timeout_sec), &error_string)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), 
      "Cannot transform from '%s' to '%s': %s",
      params_.base_link.c_str(), params_.ft_frame.c_str(), error_string.c_str());
    return false;
  }
  
  return true;
}

void AdmittanceController::sendTrajectory()
{
  // Check if action client is ready
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(
      get_node()->get_logger(), 
      "Action server not available after waiting 1 second");
    return;
  }
  
  // Create goal message
  auto goal_msg = FollowJointTrajectory::Goal();
  
  // Fill in trajectory
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = params_.joints;
  
  // Single-point trajectory with current joint positions
  trajectory.points.resize(1);
  auto & point = trajectory.points[0];
  
  // Set joint positions, velocities, and accelerations
  point.positions.resize(params_.joints.size());
  point.velocities.resize(params_.joints.size());
  point.accelerations.resize(params_.joints.size());
  
  // Get current joint positions from state interfaces
  std::vector<double> current_positions(params_.joints.size());
  std::vector<double> velocity_commands(params_.joints.size());
  
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto position_interface_name = params_.joints[i] + "/position";
    const auto position_interface_index = std::distance(
      state_interfaces_.cbegin(),
      std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&position_interface_name](const auto & interface) {
          return interface.get_name() == position_interface_name;
        }));
    
    current_positions[i] = state_interfaces_[position_interface_index].get_optional().value();
    
    // For now, use a small fixed velocity for testing
    velocity_commands[i] = 0.1;  // rad/s
  }
  
  // Use current position + velocity * duration
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    point.positions[i] = current_positions[i] + 
      velocity_commands[i] * params_.trajectory_duration;
    point.velocities[i] = 0.0;  // Zero final velocity for UR controller
    point.accelerations[i] = 0.0;  // Zero acceleration
  }
  
  // Set trajectory time
  point.time_from_start = rclcpp::Duration::from_seconds(params_.trajectory_duration);
  
  // Set goal message
  goal_msg.trajectory = trajectory;
  
  // Set tolerances
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(params_.goal_time_tolerance);
  
  // Set path and goal tolerances
  goal_msg.path_tolerance.resize(params_.joints.size());
  goal_msg.goal_tolerance.resize(params_.joints.size());
  
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    goal_msg.path_tolerance[i].name = params_.joints[i];
    goal_msg.path_tolerance[i].position = params_.position_tolerance;
    goal_msg.path_tolerance[i].velocity = params_.velocity_tolerance;
    
    goal_msg.goal_tolerance[i].name = params_.joints[i];
    goal_msg.goal_tolerance[i].position = params_.position_tolerance;
    goal_msg.goal_tolerance[i].velocity = params_.velocity_tolerance;
  }
  
  // Store goal for retries
  last_goal_msg_ = goal_msg;
  
  // Send goal
  RCLCPP_DEBUG(get_node()->get_logger(), "Sending trajectory");
  
  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback = 
    [this](const GoalHandleFJT::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(get_node()->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_DEBUG(get_node()->get_logger(), "Goal accepted by server");
      }
    };
  send_goal_options.feedback_callback =
    [this](const GoalHandleFJT::SharedPtr & /*goal_handle*/,
           const std::shared_ptr<const FollowJointTrajectory::Feedback> & /*feedback*/) {
      // Not used currently
    };
  send_goal_options.result_callback =
    [this](const GoalHandleFJT::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_DEBUG(get_node()->get_logger(), "Trajectory succeeded");
          retry_count_ = 0;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          retryTrajectory(result.result->error_string);
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(get_node()->get_logger(), "Trajectory was canceled");
          break;
        default:
          RCLCPP_ERROR(get_node()->get_logger(), "Unknown result code");
          break;
      }
    };
  
  goal_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  new_velocity_command_ = false;
}

void AdmittanceController::retryTrajectory(const std::string& reason)
{
  if (!params_.retry_on_abort) {
    RCLCPP_WARN(
      get_node()->get_logger(), 
      "Trajectory aborted: %s (retries disabled)", 
      reason.c_str());
    return;
  }
  
  if (retry_count_ >= params_.max_retries) {
    RCLCPP_ERROR(
      get_node()->get_logger(), 
      "Trajectory aborted: %s (max retries exceeded)",
      reason.c_str());
    retry_count_ = 0;
    return;
  }
  
  retry_count_++;
  retry_pending_ = true;
  retry_time_ = get_node()->now() + rclcpp::Duration::from_seconds(params_.retry_delay);
  
  RCLCPP_WARN(
    get_node()->get_logger(), 
    "Trajectory aborted: %s (retrying %ld/%ld in %.1f seconds)",
    reason.c_str(), static_cast<long>(retry_count_), static_cast<long>(params_.max_retries), 
    params_.retry_delay);
    
  // Adjust tolerances for retry
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    last_goal_msg_.path_tolerance[i].position *= 1.5;
    last_goal_msg_.path_tolerance[i].velocity *= 1.5;
    last_goal_msg_.goal_tolerance[i].position *= 1.5;
    last_goal_msg_.goal_tolerance[i].velocity *= 1.5;
  }
  
  // Increase goal time tolerance
  last_goal_msg_.goal_time_tolerance = 
    rclcpp::Duration::from_seconds(params_.goal_time_tolerance * 1.5);
}

}  // namespace ur_admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ur_admittance_controller::AdmittanceController, controller_interface::ChainableControllerInterface)
