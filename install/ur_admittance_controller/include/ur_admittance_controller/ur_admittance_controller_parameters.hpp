// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace ur_admittance_controller {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        std::string downstream_controller_name = "scaled_joint_trajectory_controller";
        std::vector<std::string> joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        std::vector<std::string> state_interfaces = {"position", "velocity"};
        std::string ft_sensor_name = "tcp_fts_sensor";
        double max_linear_velocity = 0.5;
        double max_angular_velocity = 1.0;
        bool dynamic_parameters = true;
        std::string world_frame = "world";
        std::string base_link = "base_link";
        std::string tip_link = "tool0";
        std::string ft_frame = "tool0";
        std::vector<double> joint_limits_position_min = {-6.283, -6.283, -3.142, -6.283, -6.283, -6.283};
        std::vector<double> joint_limits_position_max = {6.283, 6.283, 3.142, 6.283, 6.283, 6.283};
        std::vector<double> joint_limits_velocity_max = {3.14, 3.14, 3.14, 6.28, 6.28, 6.28};
        std::string kinematics_plugin_package = "kinematics_interface";
        std::string kinematics_plugin_name = "kinematics_interface_kdl/KinematicsInterfaceKDL";
        struct Admittance {
            std::vector<double> mass = {8.0, 8.0, 8.0, 0.8, 0.8, 0.8};
            std::vector<double> stiffness = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> damping_ratio = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
            std::vector<bool> enabled_axes = {true, true, true, true, true, true};
            double min_motion_threshold = 1.5;
            double filter_coefficient = 0.15;
            double drift_reset_threshold = 0.001;
        } admittance;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        double max_linear_velocity = 0.5;
        double max_angular_velocity = 1.0;
        bool dynamic_parameters = true;
        struct Admittance {
            double min_motion_threshold = 1.5;
            double filter_coefficient = 0.15;
            double drift_reset_threshold = 0.001;
        } admittance;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("ur_admittance_controller"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "") {
      logger_ = std::move(logger);
      prefix_ = prefix;
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.admittance.min_motion_threshold = params.admittance.min_motion_threshold;
      output.admittance.filter_coefficient = params.admittance.filter_coefficient;
      output.admittance.drift_reset_threshold = params.admittance.drift_reset_threshold;
      output.max_linear_velocity = params.max_linear_velocity;
      output.max_angular_velocity = params.max_angular_velocity;
      output.dynamic_parameters = params.dynamic_parameters;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "downstream_controller_name")) {
            updated_params.downstream_controller_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joints")) {
            updated_params.joints = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "state_interfaces")) {
            updated_params.state_interfaces = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "ft_sensor_name")) {
            updated_params.ft_sensor_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "admittance.mass")) {
            updated_params.admittance.mass = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "admittance.stiffness")) {
            updated_params.admittance.stiffness = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "admittance.damping_ratio")) {
            updated_params.admittance.damping_ratio = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "admittance.enabled_axes")) {
            updated_params.admittance.enabled_axes = param.as_bool_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "admittance.min_motion_threshold")) {
            updated_params.admittance.min_motion_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "admittance.filter_coefficient")) {
            updated_params.admittance.filter_coefficient = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "admittance.drift_reset_threshold")) {
            updated_params.admittance.drift_reset_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "max_linear_velocity")) {
            updated_params.max_linear_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "max_angular_velocity")) {
            updated_params.max_angular_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "dynamic_parameters")) {
            updated_params.dynamic_parameters = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "world_frame")) {
            updated_params.world_frame = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "base_link")) {
            updated_params.base_link = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "tip_link")) {
            updated_params.tip_link = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "ft_frame")) {
            updated_params.ft_frame = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joint_limits_position_min")) {
            updated_params.joint_limits_position_min = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joint_limits_position_max")) {
            updated_params.joint_limits_position_max = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joint_limits_velocity_max")) {
            updated_params.joint_limits_velocity_max = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "kinematics_plugin_package")) {
            updated_params.kinematics_plugin_package = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "kinematics_plugin_name")) {
            updated_params.kinematics_plugin_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "downstream_controller_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the downstream controller to chain with\n    The admittance controller will claim reference interfaces from this controller\n    This is critical for proper controller chaining architecture\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.downstream_controller_name);
          parameters_interface_->declare_parameter(prefix_ + "downstream_controller_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joints")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "List of joint names";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.joints);
          parameters_interface_->declare_parameter(prefix_ + "joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "state_interfaces")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "State interface types to claim from hardware";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.state_interfaces);
          parameters_interface_->declare_parameter(prefix_ + "state_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "ft_sensor_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the F/T sensor hardware interface";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.ft_sensor_name);
          parameters_interface_->declare_parameter(prefix_ + "ft_sensor_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "admittance.mass")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Virtual inertia matrix diagonal elements [X,Y,Z,Rx,Ry,Rz]\n    Units: [kg, kg, kg, kg⋅m², kg⋅m², kg⋅m²]\n    Range: (0.1, 100.0] for translations, (0.01, 10.0] for rotations\n    Higher values = slower response, more stable\n    Lower values = faster response, less stable\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.admittance.mass);
          parameters_interface_->declare_parameter(prefix_ + "admittance.mass", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "admittance.stiffness")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Position stiffness matrix diagonal elements [X,Y,Z,Rx,Ry,Rz]\n    Units: [N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad]\n    Range: [0.0, 2000.0] for translations, [0.0, 200.0] for rotations\n    Zero values = pure admittance control (force-to-motion)\n    Positive values = impedance control (position+force regulation)\n    \n    PRESET CONFIGURATIONS:\n    Pure Admittance (default): [0, 0, 0, 0, 0, 0]\n    Full Impedance: [100, 100, 100, 10, 10, 10]\n    XY-Compliant/Z-Stiff: [0, 0, 200, 0, 0, 0]\n    Position Control Only: [150, 150, 150, 0, 0, 0]\n    \n    TUNING GUIDE:\n    0 N/m: Pure compliance (no return)\n    10-50 N/m: Very soft return\n    50-200 N/m: Moderate stiffness\n    200-500 N/m: Firm return\n    500-2000 N/m: Very stiff\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.admittance.stiffness);
          parameters_interface_->declare_parameter(prefix_ + "admittance.stiffness", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "admittance.damping_ratio")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Damping ratio coefficients [X,Y,Z,Rx,Ry,Rz]\n    Units: [dimensionless]\n    Range: [0.1, 2.0] (typical)\n    Values < 1.0: Underdamped (some oscillation, faster response)\n    Values = 1.0: Critically damped (no oscillation, optimal response)\n    Values > 1.0: Overdamped (no oscillation, slower response)\n    Used to compute D matrix based on M and K: D = 2ζ√(MK) if K>0\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.admittance.damping_ratio);
          parameters_interface_->declare_parameter(prefix_ + "admittance.damping_ratio", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "admittance.enabled_axes")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable/disable admittance control for each axis [X,Y,Z,Rx,Ry,Rz]\n    When disabled, the controller will maintain position in that axis\n    Common configurations:\n    - [true, true, true, false, false, false]: XYZ translation only\n    - [false, false, true, false, false, false]: Z-axis (vertical) compliance only\n    - [true, true, false, false, false, true]: Horizontal plane + rotation around Z\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.admittance.enabled_axes);
          parameters_interface_->declare_parameter(prefix_ + "admittance.enabled_axes", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "admittance.min_motion_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Minimum force/torque magnitude to trigger motion (deadband)\n    Units: Newtons (for forces) / Nm (for torques)\n    Range: [0.1, 10.0] (typical)\n    Smaller values: More sensitive to forces/torques (may detect noise)\n    Larger values: Less sensitive (requires more deliberate force)\n    Used to filter out sensor noise and unintentional contacts\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.admittance.min_motion_threshold);
          parameters_interface_->declare_parameter(prefix_ + "admittance.min_motion_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "admittance.filter_coefficient")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Low-pass filter coefficient for wrench signals\n    Units: [dimensionless]\n    Range: [0.01, 0.9] (typical)\n    Formula: filtered = (1-coeff)*previous + coeff*current\n    Smaller values: More filtering, smoother motion, more lag\n    Larger values: Less filtering, more responsive, may amplify noise\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.admittance.filter_coefficient);
          parameters_interface_->declare_parameter(prefix_ + "admittance.filter_coefficient", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "admittance.drift_reset_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Velocity threshold for position drift prevention\n    Units: meters/second\n    Range: [0.0001, 0.01] (typical)\n    When Cartesian velocity falls below this threshold, the controller\n    will automatically update reference position to prevent drift.\n    Smaller values: More accurate positioning, may detect noise\n    Larger values: Less sensitive, more motion required to reset\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.admittance.drift_reset_threshold);
          parameters_interface_->declare_parameter(prefix_ + "admittance.drift_reset_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "max_linear_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum linear velocity in Cartesian space\n    Units: meters/second\n    Range: [0.1, 1.0] (typical, robot-dependent)\n    Critical safety parameter that limits the maximum translational\n    velocity regardless of external forces. Should be set based on\n    workspace constraints and safety requirements.\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.max_linear_velocity);
          parameters_interface_->declare_parameter(prefix_ + "max_linear_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "max_angular_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum angular velocity in Cartesian space\n    Units: radians/second\n    Range: [0.2, 2.0] (typical, robot-dependent)\n    Critical safety parameter that limits the maximum rotational\n    velocity regardless of external torques. Should be set based on\n    workspace constraints and safety requirements.\n    ";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.max_angular_velocity);
          parameters_interface_->declare_parameter(prefix_ + "max_angular_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "dynamic_parameters")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable live parameter tuning";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.dynamic_parameters);
          parameters_interface_->declare_parameter(prefix_ + "dynamic_parameters", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "world_frame")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "World reference frame";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.world_frame);
          parameters_interface_->declare_parameter(prefix_ + "world_frame", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "base_link")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Robot base link frame";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.base_link);
          parameters_interface_->declare_parameter(prefix_ + "base_link", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "tip_link")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Robot end-effector frame";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.tip_link);
          parameters_interface_->declare_parameter(prefix_ + "tip_link", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "ft_frame")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "F/T sensor frame";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.ft_frame);
          parameters_interface_->declare_parameter(prefix_ + "ft_frame", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joint_limits_position_min")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Minimum joint position limits (radians)";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.joint_limits_position_min);
          parameters_interface_->declare_parameter(prefix_ + "joint_limits_position_min", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joint_limits_position_max")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum joint position limits (radians)";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.joint_limits_position_max);
          parameters_interface_->declare_parameter(prefix_ + "joint_limits_position_max", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joint_limits_velocity_max")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum joint velocity limits (rad/s)";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.joint_limits_velocity_max);
          parameters_interface_->declare_parameter(prefix_ + "joint_limits_velocity_max", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "kinematics_plugin_package")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Package containing the kinematics plugin";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.kinematics_plugin_package);
          parameters_interface_->declare_parameter(prefix_ + "kinematics_plugin_package", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "kinematics_plugin_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the kinematics plugin to use";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.kinematics_plugin_name);
          parameters_interface_->declare_parameter(prefix_ + "kinematics_plugin_name", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "downstream_controller_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.downstream_controller_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "joints");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joints = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "state_interfaces");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.state_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "ft_sensor_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.ft_sensor_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "admittance.mass");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.admittance.mass = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "admittance.stiffness");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.admittance.stiffness = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "admittance.damping_ratio");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.admittance.damping_ratio = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "admittance.enabled_axes");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.admittance.enabled_axes = param.as_bool_array();
      param = parameters_interface_->get_parameter(prefix_ + "admittance.min_motion_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.admittance.min_motion_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "admittance.filter_coefficient");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.admittance.filter_coefficient = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "admittance.drift_reset_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.admittance.drift_reset_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "max_linear_velocity");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.max_linear_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "max_angular_velocity");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.max_angular_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "dynamic_parameters");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.dynamic_parameters = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "world_frame");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.world_frame = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "base_link");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.base_link = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "tip_link");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.tip_link = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "ft_frame");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.ft_frame = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "joint_limits_position_min");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joint_limits_position_min = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "joint_limits_position_max");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joint_limits_position_max = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "joint_limits_velocity_max");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joint_limits_velocity_max = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "kinematics_plugin_package");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.kinematics_plugin_package = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "kinematics_plugin_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.kinematics_plugin_name = param.as_string();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

      // rclcpp::Logger cannot be default-constructed
      // so we must provide a initialization here even though
      // every one of our constructors initializes logger_
      rclcpp::Logger logger_ = rclcpp::get_logger("ur_admittance_controller");
      std::mutex mutable mutex_;
  };

} // namespace ur_admittance_controller
