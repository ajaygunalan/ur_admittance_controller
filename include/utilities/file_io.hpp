#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

#include <utilities/error.hpp>

namespace ur_admittance_controller {
namespace file_io {

// Load config file from package config directory with error handling
// Looks for files in: $ROS_WORKSPACE/src/ur_admittance_controller/config/
// or defaults to: $HOME/ros2_ws/src/ur_admittance_controller/config/
Result<YAML::Node> LoadConfigFile(const std::string& filename);

} // namespace file_io
} // namespace ur_admittance_controller