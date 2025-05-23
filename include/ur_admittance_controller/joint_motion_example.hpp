/**
 * @file joint_motion_example.hpp
 * @brief Example for executing joint trajectories on UR robots in Gazebo
 * 
 * Provides a ROS2 action client that sends joint trajectories to Universal Robots
 * manipulators and monitors execution progress. Supports pre-defined trajectories
 * and timeout handling.
 */

 #ifndef JOINT_MOTION_EXAMPLE_HPP_
 #define JOINT_MOTION_EXAMPLE_HPP_
 
 #include <rclcpp/rclcpp.hpp>
 #include <rclcpp_action/rclcpp_action.hpp>
 #include <control_msgs/action/follow_joint_trajectory.hpp>
 #include <trajectory_msgs/msg/joint_trajectory.hpp>
 #include <trajectory_msgs/msg/joint_trajectory_point.hpp>
 #include <std_srvs/srv/trigger.hpp>
 #include <array>
 #include <chrono>
 #include <memory>
 #include <string>
 #include <vector>
 #include <map>
 
 namespace ur_admittance {
 
 using FollowJT = control_msgs::action::FollowJointTrajectory;
 using GoalHandleJT = rclcpp_action::ClientGoalHandle<FollowJT>;
 
 /**
  * @brief Represents a single point in a joint trajectory
  */
 struct TrajectoryPoint {
   std::array<double,6> positions;
   std::array<double,6> velocities;
   rclcpp::Duration time_from_start;
 
   TrajectoryPoint(
     const std::array<double,6>& pos,
     const std::array<double,6>& vel,
     const rclcpp::Duration& tfs);
 
   trajectory_msgs::msg::JointTrajectoryPoint to_msg() const;
 };
 
 /**
  * @brief Represents a complete joint trajectory
  */
 struct Trajectory {
   std::string name;
   std::vector<TrajectoryPoint> points;
 
   trajectory_msgs::msg::JointTrajectory to_msg(
     const std::vector<std::string>& joint_names) const;
 };
 
 /**
  * @brief Client for executing joint trajectories
  * 
  * This node provides a client for executing pre-defined joint trajectories
  * on UR robots. It supports trajectory sequencing, progress monitoring,
  * and cancellation.
  */
 class JointMotionExample : public rclcpp::Node
 {
 public:
   JointMotionExample();
 
 private:
   void initialize_trajectories();
   void execute_next();
   void send_goal(const Trajectory& traj);
   void cancel_callback(
     const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
     std::shared_ptr<std_srvs::srv::Trigger::Response> res);
 
   // ROS interfaces
   rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
   rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
   rclcpp::TimerBase::SharedPtr timeout_timer_;
   rclcpp::TimerBase::SharedPtr next_timer_;
   GoalHandleJT::SharedPtr active_goal_handle_;
 
   // Parameters
   std::string controller_name_;
   std::vector<std::string> joint_names_;
   double trajectory_timeout_;
   double position_tolerance_;
   double velocity_tolerance_;
   double path_tolerance_;
 
   // Trajectory data
   std::vector<Trajectory> trajectories_;
   size_t current_index_;
 };
 
 }  // namespace ur_admittance
 
 #endif  // JOINT_MOTION_EXAMPLE_HPP_