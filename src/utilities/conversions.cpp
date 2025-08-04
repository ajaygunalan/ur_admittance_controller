#include <utilities/conversions.hpp>

namespace ur_admittance_controller {
namespace conversions {

geometry_msgs::msg::WrenchStamped ToMsg(
    const Wrench6d& wrench,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    FillMsg(wrench, msg);
    return msg;
}

}
}