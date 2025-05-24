#!/usr/bin/env python3
"""
Controller Status Checker
Monitors the status of a specific controller (e.g., ur_admittance_controller)

Usage:
  ros2 run ur_admittance_controller controller_status.py
  ros2 run ur_admittance_controller controller_status.py --ros-args -p controller_name:=ur_admittance_controller
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist, WrenchStamped
from sensor_msgs.msg import JointState
import time


class ControllerStatusChecker(Node):
    def __init__(self):
        super().__init__('controller_status_checker')
        
        # Parameters
        self.declare_parameter('controller_name', 'ur_admittance_controller')
        self.declare_parameter('expected_state', 'active')
        self.declare_parameter('check_period', 5.0)
        
        self.controller_name = self.get_parameter('controller_name').value
        self.expected_state = self.get_parameter('expected_state').value
        self.check_period = self.get_parameter('check_period').value
        
        # Service clients
        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        
        # Subscribers for data monitoring
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.cart_vel_sub = self.create_subscription(
            Twist, f'/{self.controller_name}/cartesian_velocity_command',
            self.cart_vel_callback, 10
        )
        
        self.ft_sub = self.create_subscription(
            WrenchStamped, '/ft_sensor_readings',
            self.ft_callback, 10
        )
        
        # Status tracking
        self.joint_state_received = False
        self.cart_vel_received = False
        self.ft_received = False
        self.last_check_time = time.time()
        self.controller_active = False
        
        # Timer for periodic checks
        self.status_timer = self.create_timer(self.check_period, self.check_controller_status)
        
        self.get_logger().info(f"🔍 Monitoring controller: {self.controller_name}")
        self.get_logger().info(f"📊 Expected state: {self.expected_state}")

    def joint_state_callback(self, msg):
        self.joint_state_received = True

    def cart_vel_callback(self, msg):
        self.cart_vel_received = True
        
        # Check if controller is generating motion commands
        velocity_magnitude = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        angular_magnitude = (msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)**0.5
        
        if velocity_magnitude > 0.001 or angular_magnitude > 0.001:
            self.get_logger().info(
                f"🎯 {self.controller_name} generating motion - "
                f"Linear: {velocity_magnitude:.3f} m/s, Angular: {angular_magnitude:.3f} rad/s",
                throttle_duration_sec=2.0
            )

    def ft_callback(self, msg):
        self.ft_received = True
        
        # Check F/T sensor activity
        force_magnitude = (msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)**0.5
        torque_magnitude = (msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.torque.z**2)**0.5
        
        if force_magnitude > 1.0 or torque_magnitude > 0.5:
            self.get_logger().info(
                f"🔧 F/T sensor active - Force: {force_magnitude:.1f}N, Torque: {torque_magnitude:.1f}Nm",
                throttle_duration_sec=1.0
            )

    def check_controller_status(self):
        """Check the status of the specific controller"""
        
        if not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("❌ Controller Manager not available")
            return
        
        # Get controller list
        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is None:
            self.get_logger().error("❌ Failed to get controller list")
            return
        
        controllers = future.result().controller
        
        # Find our controller
        controller_found = False
        controller_state = "not_found"
        
        for controller in controllers:
            if controller.name == self.controller_name:
                controller_found = True
                controller_state = controller.state
                self.controller_active = (controller_state == "active")
                break
        
        # Report status
        current_time = self.get_current_time()
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🔍 CONTROLLER STATUS CHECK - {current_time}")
        self.get_logger().info("=" * 60)
        
        if controller_found:
            status_icon = "✅" if controller_state == self.expected_state else "❌"
            self.get_logger().info(f"{status_icon} {self.controller_name}: {controller_state}")
            
            if controller_state != self.expected_state:
                self.get_logger().warn(f"⚠️  Expected '{self.expected_state}' but got '{controller_state}'")
                
                if controller_state == "inactive" and self.expected_state == "active":
                    self.get_logger().info("💡 To activate: ros2 control set_controller_state {} start".format(self.controller_name))
                elif controller_state == "active" and self.expected_state == "inactive":
                    self.get_logger().info("💡 To deactivate: ros2 control set_controller_state {} stop".format(self.controller_name))
        else:
            self.get_logger().error(f"❌ {self.controller_name}: NOT FOUND")
            self.get_logger().info("💡 Available controllers:")
            for controller in controllers:
                self.get_logger().info(f"   - {controller.name} [{controller.state}]")
        
        # Check data flow
        self.get_logger().info("📡 DATA FLOW STATUS:")
        self.get_logger().info(f"   Joint States:     {'✅' if self.joint_state_received else '❌'}")
        self.get_logger().info(f"   Cartesian Vel:    {'✅' if self.cart_vel_received else '❌'}")
        self.get_logger().info(f"   F/T Sensor:       {'✅' if self.ft_received else '❌'}")
        
        # Provide guidance
        if controller_found and controller_state == "active":
            if not self.ft_received:
                self.get_logger().warn("⚠️  No F/T sensor data - check sensor configuration")
            elif self.ft_received and not self.cart_vel_received:
                self.get_logger().info("🎯 F/T sensor working - apply force to test admittance")
            elif self.ft_received and self.cart_vel_received:
                self.get_logger().info("🎉 Controller fully operational!")
            
            self.get_logger().info("🧪 TESTING COMMANDS:")
            self.get_logger().info("   Check F/T: ros2 topic echo /ft_sensor_readings --once")
            self.get_logger().info("   Check velocity: ros2 topic echo /{}/cartesian_velocity_command".format(self.controller_name))
            self.get_logger().info("   Live tuning: ros2 param set /{} mass.0 5.0".format(self.controller_name))
        
        self.get_logger().info("=" * 60)

    def get_current_time(self):
        """Get formatted current time"""
        return time.strftime("%H:%M:%S", time.localtime())


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ControllerStatusChecker()
        node.get_logger().info("🚀 Controller status checker started")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info("👋 Controller status checker shutting down")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()