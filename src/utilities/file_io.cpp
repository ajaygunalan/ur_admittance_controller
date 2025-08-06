#include <utilities/file_io.hpp>
#include <filesystem>
#include <rclcpp/logging.hpp>
#include <fmt/core.h>

namespace ur_admittance_controller {
namespace file_io {

Result<YAML::Node> LoadConfigFile(const std::string& filename) {
    const char* workspace_env = std::getenv("ROS_WORKSPACE");
    std::string workspace = workspace_env ? workspace_env :
                           std::string(std::getenv("HOME")) + "/ros2_ws";

    std::filesystem::path config_path =
        std::filesystem::path(workspace) / "src" / "ur_admittance_controller" / "config" / filename;

    RCLCPP_DEBUG(rclcpp::get_logger("file_io"),
                 "Loading config from: %s", config_path.c_str());

    if (!std::filesystem::exists(config_path)) {
        return tl::unexpected(MakeError(ErrorCode::kFileNotFound,
            fmt::format("Config file not found: {}", config_path.string())));
    }

    try {
        return YAML::LoadFile(config_path.string());
    } catch (const YAML::Exception& e) {
        return tl::unexpected(MakeError(ErrorCode::kInvalidConfiguration,
            fmt::format("Failed to parse YAML: {}", e.what())));
    }
}

std::filesystem::path GetConfigPath(const std::string& filename) {
    const char* workspace_env = std::getenv("ROS_WORKSPACE");
    std::string workspace = workspace_env ? workspace_env :
                           std::string(std::getenv("HOME")) + "/ros2_ws";
    
    return std::filesystem::path(workspace) / "src" / "ur_admittance_controller" / "config" / filename;
}

} // namespace file_io
} // namespace ur_admittance_controller