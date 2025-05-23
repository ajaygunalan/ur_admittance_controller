#include "ur_admittance_controller/wrench_signal_generator.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/wrench.hpp>

namespace ur_admittance {

using namespace std::chrono_literals;

WrenchSignalGenerator::WrenchSignalGenerator(const rclcpp::NodeOptions& options)
: Node("wrench_signal_generator", options), time_(0.0)
{
  // Declare and load parameters
  declare_parameter("frequency", 0.5);               // Hz
  declare_parameter("amplitude", 10.0);              // N
  declare_parameter("publish_rate", 100.0);          // Hz
  declare_parameter("tool_frame_id", "tool0");      // Tool frame
  declare_parameter("base_frame_id", "base_link");   // Base frame
  declare_parameter("tool_mass", 0.5);               // Tool mass in kg
  declare_parameter("tool_offset_x", 0.0);           // Tool COM offset X (m)
  declare_parameter("tool_offset_y", 0.0);           // Tool COM offset Y (m)
  declare_parameter("tool_offset_z", 0.05);          // Tool COM offset Z (m)
  declare_parameter("apply_gravity_comp", true);     // Apply gravity compensation
  
  // Load parameters
  frequency_ = get_parameter("frequency").as_double();
  amplitude_ = get_parameter("amplitude").as_double();
  publish_rate_ = get_parameter("publish_rate").as_double();
  tool_frame_id_ = get_parameter("tool_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();
  tool_mass_ = get_parameter("tool_mass").as_double();
  apply_gravity_comp_ = get_parameter("apply_gravity_comp").as_bool();
  
  // Set tool offset (center of mass relative to tool frame)
  tool_offset_ = Eigen::Vector3d(
    get_parameter("tool_offset_x").as_double(),
    get_parameter("tool_offset_y").as_double(),
    get_parameter("tool_offset_z").as_double());

  // Setup TF listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Create publisher, service, and timer
  pub_wrench_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/wrench_fake", 10);
  
  zero_service_ = create_service<std_srvs::srv::Trigger>(
    "zero_ftsensor",
    std::bind(&WrenchSignalGenerator::zero_callback, this, std::placeholders::_1, std::placeholders::_2));
  
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_),
    std::bind(&WrenchSignalGenerator::timer_callback, this));

  RCLCPP_INFO(get_logger(), "Enhanced WrenchSignalGenerator initialized:");
  RCLCPP_INFO(get_logger(), "  - Frequency: %.2f Hz", frequency_);
  RCLCPP_INFO(get_logger(), "  - Amplitude: %.2f N", amplitude_);
  RCLCPP_INFO(get_logger(), "  - Tool frame: %s", tool_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "  - Base frame: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "  - Tool mass: %.2f kg", tool_mass_);
  RCLCPP_INFO(get_logger(), "  - Gravity compensation: %s", apply_gravity_comp_ ? "enabled" : "disabled");
}

void WrenchSignalGenerator::timer_callback()
{
  try {
    // Generate wrench in tool frame
    Eigen::Matrix<double, 6, 1> wrench_tool = generateWrench();
    
    // Apply gravity compensation if enabled
    if (apply_gravity_comp_) {
      wrench_tool = applyGravityCompensation(wrench_tool);
    }
    
    // Transform to base frame (like real UR5e sensors)
    Eigen::Matrix<double, 6, 1> wrench_base = transformWrench(wrench_tool);
    
    // Create and publish message
    auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    msg->header.stamp = get_clock()->now();
    msg->header.frame_id = base_frame_id_;
    
    // Fill in wrench data (measured at TCP, expressed in base frame)
    msg->wrench.force.x = wrench_base(0);
    msg->wrench.force.y = wrench_base(1);
    msg->wrench.force.z = wrench_base(2);
    msg->wrench.torque.x = wrench_base(3);
    msg->wrench.torque.y = wrench_base(4);
    msg->wrench.torque.z = wrench_base(5);
    
    pub_wrench_->publish(std::move(msg));
    
    // Update time
    time_ += 1.0 / publish_rate_;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(), "Could not transform wrench: %s", ex.what());
    
    // Fallback: publish just the raw wrench in tool frame
    auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    msg->header.stamp = get_clock()->now();
    msg->header.frame_id = tool_frame_id_;
    msg->wrench.force.x = amplitude_ * std::sin(2.0 * M_PI * frequency_ * time_);
    pub_wrench_->publish(std::move(msg));
    
    // Still update time
    time_ += 1.0 / publish_rate_;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "Error in timer_callback: %s", ex.what());
  }
}

// Zero the force/torque sensor (simulate the zero_ftsensor() function of UR robots)
void WrenchSignalGenerator::zero_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    // Generate the current wrench to capture the current offset
    Eigen::Matrix<double, 6, 1> current_wrench = generateWrench();
    
    // Save this as the new offset to subtract
    wrench_offset_ = Eigen::Vector3d(current_wrench(0), current_wrench(1), current_wrench(2));
    
    response->success = true;
    response->message = "F/T sensor successfully zeroed";
    RCLCPP_INFO(get_logger(), "F/T sensor zeroed");
  } catch (const std::exception& ex) {
    response->success = false;
    response->message = std::string("Failed to zero F/T sensor: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "Failed to zero F/T sensor: %s", ex.what());
  }
}

// Generate a simulated wrench in the tool frame
Eigen::Matrix<double, 6, 1> WrenchSignalGenerator::generateWrench()
{
  // Generate a sinusoidal force in the tool's x-axis
  double force_x = amplitude_ * std::sin(2.0 * M_PI * frequency_ * time_);
  
  // Subtract any offset from zeroing
  force_x -= wrench_offset_.x();
  
  // Create the full wrench vector (force and torque)
  Eigen::Matrix<double, 6, 1> wrench;
  wrench << force_x, 0.0, 0.0, 0.0, 0.0, 0.0;
  
  return wrench;
}

// Apply gravity compensation to the wrench
Eigen::Matrix<double, 6, 1> WrenchSignalGenerator::applyGravityCompensation(
  const Eigen::Matrix<double, 6, 1>& wrench)
{
  try {
    // Get the tool orientation from TF
    geometry_msgs::msg::TransformStamped transform = 
      tf_buffer_->lookupTransform(base_frame_id_, tool_frame_id_, tf2::TimePointZero);
    
    // Convert to tf2 matrix
    tf2::Quaternion q;
    q.setValue(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    tf2::Matrix3x3 rot_matrix(q);
    
    // Calculate gravity vector in tool frame (transpose of rotation matrix = inverse for pure rotation)
    tf2::Vector3 gravity_tf(gravity_.x(), gravity_.y(), gravity_.z());
    tf2::Vector3 gravity_tool = tf2::quatRotate(q.inverse(), gravity_tf);
    
    // Calculate force due to tool mass
    Eigen::Vector3d force_gravity(
      tool_mass_ * gravity_tool.x(),
      tool_mass_ * gravity_tool.y(),
      tool_mass_ * gravity_tool.z());
    
    // Calculate torque due to offset center of mass
    Eigen::Vector3d torque_gravity = tool_offset_.cross(force_gravity);
    
    // Create a new wrench with gravity compensated
    Eigen::Matrix<double, 6, 1> compensated_wrench = wrench;
    compensated_wrench(0) -= force_gravity.x();
    compensated_wrench(1) -= force_gravity.y();
    compensated_wrench(2) -= force_gravity.z();
    compensated_wrench(3) -= torque_gravity.x();
    compensated_wrench(4) -= torque_gravity.y();
    compensated_wrench(5) -= torque_gravity.z();
    
    return compensated_wrench;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(), "Could not apply gravity compensation: %s", ex.what());
    return wrench; // Return original wrench if transformation fails
  }
}

// Transform wrench from tool frame to base frame (like real UR5e sensors)
Eigen::Matrix<double, 6, 1> WrenchSignalGenerator::transformWrench(
  const Eigen::Matrix<double, 6, 1>& wrench_tool)
{
  // Get the transform from tool frame to base frame
  geometry_msgs::msg::TransformStamped transform = 
    tf_buffer_->lookupTransform(base_frame_id_, tool_frame_id_, tf2::TimePointZero);
  
  // Extract rotation and translation
  tf2::Quaternion q;
  q.setValue(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w);
  
  tf2::Vector3 p(transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);
  
  // Extract force and torque components
  tf2::Vector3 force_tool(wrench_tool(0), wrench_tool(1), wrench_tool(2));
  tf2::Vector3 torque_tool(wrench_tool(3), wrench_tool(4), wrench_tool(5));
  
  // Transform force (rotate)
  tf2::Vector3 force_base = tf2::quatRotate(q, force_tool);
  
  // Transform torque (rotate and account for lever arm)
  tf2::Vector3 torque_base = tf2::quatRotate(q, torque_tool) + p.cross(force_base);
  
  // Combine into new wrench
  Eigen::Matrix<double, 6, 1> wrench_base;
  wrench_base << force_base.x(), force_base.y(), force_base.z(),
                 torque_base.x(), torque_base.y(), torque_base.z();
  
  return wrench_base;
}

} // namespace ur_admittance

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ur_admittance::WrenchSignalGenerator>());
  rclcpp::shutdown();
  return 0;
}