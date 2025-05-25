#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_SERVICES_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_SERVICES_HPP_

#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ur_admittance_controller
{

/**
 * @brief Service interface for dynamic desired pose manipulation
 *
 * This header adds services for manipulating the desired pose in impedance mode:
 * - reset_desired_pose: Sets the desired pose to the current pose (identity error)
 * - set_desired_pose: Sets the desired pose to a specified value
 */
class AdmittanceControllerServices
{
public:
  using TriggerService = std_srvs::srv::Trigger;

protected:
  // Service interfaces
  rclcpp::Service<TriggerService>::SharedPtr reset_pose_service_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr set_pose_sub_;
  
  // Publishers for current state
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_;

  // Service callback
  void handle_reset_pose(
    const std::shared_ptr<TriggerService::Request> request,
    std::shared_ptr<TriggerService::Response> response);
    
  // Subscription callback
  void handle_set_desired_pose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_SERVICES_HPP_
