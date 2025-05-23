/**
 * @file joint_motion_example.cpp
 * @brief Implementation of joint motion examples for UR robots
 * 
 * Executes a sequence of pre-defined trajectories on UR robots in Gazebo.
 * Provides progress monitoring, timeout handling, and cancellation support.
 */

 #include "ur_admittance_controller/joint_motion_example.hpp"
 #include <rclcpp/logging.hpp>
 
 namespace ur_admittance {
 
 using namespace std::chrono_literals;
 
 // Implementation of trajectory point with position, velocity and timing data
 TrajectoryPoint::TrajectoryPoint(
   const std::array<double,6>& pos,
   const std::array<double,6>& vel,
   const rclcpp::Duration& tfs)
 : positions(pos), velocities(vel), time_from_start(tfs) {}
 
 trajectory_msgs::msg::JointTrajectoryPoint TrajectoryPoint::to_msg() const {
   trajectory_msgs::msg::JointTrajectoryPoint pt;
   pt.positions = std::vector<double>(positions.begin(), positions.end());
   pt.velocities = std::vector<double>(velocities.begin(), velocities.end());
   
   // Convert to message-compatible duration format
   builtin_interfaces::msg::Duration builtin_duration;
   builtin_duration.sec = static_cast<int32_t>(time_from_start.seconds());
   builtin_duration.nanosec = static_cast<uint32_t>(
       time_from_start.nanoseconds() % 1000000000);
   pt.time_from_start = builtin_duration;
   
   return pt;
 }
 
 // Convert trajectory to ROS message format with provided joint names
 trajectory_msgs::msg::JointTrajectory Trajectory::to_msg(
   const std::vector<std::string>& joint_names) const
 {
   trajectory_msgs::msg::JointTrajectory traj;
   traj.joint_names = joint_names;
   for (const auto& pt : points) {
     traj.points.push_back(pt.to_msg());
   }
   return traj;
 }
 
 
 JointMotionExample::JointMotionExample()
 : Node("joint_motion_example"), current_index_(0)
 {
   // Declare parameters
   declare_parameter("controller_name", "scaled_joint_trajectory_controller");
   declare_parameter<std::vector<std::string>>("joints", {
     "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
   });
   declare_parameter("trajectory_timeout", 30.0);
   declare_parameter("position_tolerance", 0.01);
   declare_parameter("velocity_tolerance", 0.01);
   declare_parameter("path_tolerance", 0.0);
 
   // Load parameters
   controller_name_ = get_parameter("controller_name").as_string() + "/follow_joint_trajectory";
   joint_names_ = get_parameter("joints").as_string_array();
   trajectory_timeout_ = get_parameter("trajectory_timeout").as_double();
   position_tolerance_ = get_parameter("position_tolerance").as_double();
   velocity_tolerance_ = get_parameter("velocity_tolerance").as_double();
   path_tolerance_ = get_parameter("path_tolerance").as_double();
 
   if (joint_names_.empty()) {
     RCLCPP_ERROR(get_logger(), "Parameter 'joints' cannot be empty");
     throw std::runtime_error("Parameter 'joints' cannot be empty");
   }
 
   // Setup cancellation service
   cancel_srv_ = create_service<std_srvs::srv::Trigger>(
     "cancel_trajectory",
     std::bind(&JointMotionExample::cancel_callback, this, std::placeholders::_1,
               std::placeholders::_2));
 
   // Create trajectories
   initialize_trajectories();
   if (trajectories_.empty()) {
     RCLCPP_ERROR(get_logger(), "No trajectories defined");
     throw std::runtime_error("No trajectories defined");
   }
 
   // Connect to controller
   action_client_ = rclcpp_action::create_client<FollowJT>(this, controller_name_);
   RCLCPP_INFO(get_logger(), "Waiting for action server '%s'...",
     controller_name_.c_str());
   if (!action_client_->wait_for_action_server(10s)) {
     RCLCPP_ERROR(get_logger(), "Action server not available");
     throw std::runtime_error("Action server not available");
   }
 
   // Begin execution
   execute_next();
 }
 
 void JointMotionExample::initialize_trajectories() {
   trajectories_ = {
     // Home to ready pose
     {"home_to_ready", {
       // Starting position
       {
         {0.043128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18},
         {0, 0, 0, 0, 0, 0}, 
         rclcpp::Duration(4, 0)
       },
       // Ready position
       {
         {-0.195016, -1.70093, 0.902027, -0.944217, -1.52982, -0.195171},
         {0, 0, 0, 0, 0, 0}, 
         rclcpp::Duration(8, 0)
       }
     }},
     // Ready to approach position
     {"ready_to_approach", {
       // Ready position (matching end of previous trajectory)
       {
         {-0.195016, -1.70094, 0.902027, -0.944217, -1.52982, -0.195171},
         {0, 0, 0, 0, 0, 0}, 
         rclcpp::Duration(0, 0)
       },
       // Approach position
       {
         {0.30493, -0.982258, 0.955637, -1.48215, -1.72737, 0.204445},
         {0, 0, 0, 0, 0, 0}, 
         rclcpp::Duration(8, 0)
       }
     }}
   };
   RCLCPP_INFO(get_logger(), "Using %zu built-in trajectories", trajectories_.size());
 }
 
 // Execute next trajectory in sequence or shut down if complete
 void JointMotionExample::execute_next() {
   if (current_index_ >= trajectories_.size()) {
     RCLCPP_INFO(get_logger(), "All trajectories completed");
     rclcpp::shutdown();
     return;
   }
 
   auto& traj = trajectories_[current_index_];
   RCLCPP_INFO(get_logger(), "Starting trajectory %zu/%zu: %s",
     current_index_ + 1, trajectories_.size(), traj.name.c_str());
   send_goal(traj);
   current_index_++;
 }
 
 // Send trajectory to action server with appropriate callbacks and tolerances
 void JointMotionExample::send_goal(const Trajectory& traj) {
   auto goal_msg = FollowJT::Goal();
   goal_msg.trajectory = traj.to_msg(joint_names_);
   goal_msg.goal_time_tolerance.sec = 0;
   goal_msg.goal_time_tolerance.nanosec = 500000000;  // 0.5 seconds
 
   // Set tolerances for each joint
   for (const auto& joint : joint_names_) {
     control_msgs::msg::JointTolerance tol;
     tol.name = joint;
     tol.position = position_tolerance_;
     tol.velocity = velocity_tolerance_;
     goal_msg.goal_tolerance.push_back(tol);
 
     control_msgs::msg::JointTolerance path_tol;
     path_tol.name = joint;
     path_tol.position = path_tolerance_;
     path_tol.velocity = velocity_tolerance_;
     goal_msg.path_tolerance.push_back(path_tol);
   }
 
   // Configure action callbacks
   auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();
 
   // Goal response callback
   opts.goal_response_callback =
     [this](const rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr& goal_handle) {
       active_goal_handle_ = goal_handle;
       
       if (!active_goal_handle_) {
         RCLCPP_ERROR(get_logger(), "Goal rejected");
         rclcpp::shutdown();
         return;
       }
       
       RCLCPP_INFO(get_logger(), "Goal accepted");
       
       // Setup timeout timer
       timeout_timer_ = create_wall_timer(
         std::chrono::duration<double>(trajectory_timeout_),
         [this]() {
           if (active_goal_handle_) {
             RCLCPP_WARN(get_logger(), "Timeout, canceling goal");
             action_client_->async_cancel_goal(active_goal_handle_);
           }
           timeout_timer_->cancel();
         }
       );
     };
 
   // Feedback callback
   opts.feedback_callback =
     [this](auto, const std::shared_ptr<const FollowJT::Feedback> feedback) {
       auto& pts = trajectories_[current_index_ - 1].points;
       const auto& goal = pts.back().positions;
       const auto& actual = feedback->actual.positions;
       double sum = 0, total = 0;
       for (size_t i = 0; i < goal.size(); ++i) {
         sum += std::abs(actual[i] - goal[i]);
         total += std::abs(goal[i]);
       }
       double progress = (total > 0) ? (1.0 - sum/total) * 100.0 : 0.0;
       progress = std::clamp(progress, 0.0, 100.0);
       RCLCPP_INFO(get_logger(), "Progress: %.1f%%", progress);
     };
 
   // Result callback
   opts.result_callback =
     [this](const GoalHandleJT::WrappedResult& res) {
       timeout_timer_->cancel();
       switch (res.code) {
         case rclcpp_action::ResultCode::SUCCEEDED:
           RCLCPP_INFO(get_logger(), "Trajectory succeeded");
           // Queue next trajectory with delay
           next_timer_ = create_wall_timer(
             2s,
             [this]() {
               next_timer_->cancel();
               execute_next();
             }
           );
           break;
 
         case rclcpp_action::ResultCode::ABORTED:
           RCLCPP_ERROR(get_logger(), "Trajectory aborted: %s",
             res.result->error_string.c_str());
           rclcpp::shutdown();
           break;
 
         case rclcpp_action::ResultCode::CANCELED:
           RCLCPP_WARN(get_logger(), "Trajectory canceled");
           rclcpp::shutdown();
           break;
 
         default:
           RCLCPP_ERROR(get_logger(), "Unknown result code");
           rclcpp::shutdown();
       }
     };
 
   // Send goal
   action_client_->async_send_goal(goal_msg, opts);
 }
 
 // Handle external cancellation requests
 void JointMotionExample::cancel_callback(
   const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
 {
   if (active_goal_handle_) {
     action_client_->async_cancel_goal(active_goal_handle_);
     res->success = true;
     res->message = "Trajectory canceled by user";
   } else {
     res->success = false;
     res->message = "No active trajectory to cancel";
   }
 }
 
 }  // namespace ur_admittance
 
 /**
  * @brief Initialize and run the joint motion example
  */
 int main(int argc, char * argv[]) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<ur_admittance::JointMotionExample>());
   rclcpp::shutdown();
   return 0;
 }