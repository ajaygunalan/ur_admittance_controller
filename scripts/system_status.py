#!/usr/bin/env python3
"""
Minimal UR Admittance Control System Status Checker
Quick check of controllers and data flow with simple output

Usage:
  ros2 run ur_admittance_controller system_status.py
  ros2 run ur_admittance_controller system_status.py --ros-args -p focus_controller:=ur_admittance_controller
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist, WrenchStamped
from sensor_msgs.msg import JointState


class SystemStatusChecker(Node):
    def __init__(self):
        super().__init__('system_status_checker')
        
        # Parameters
        self.declare_parameter('check_period', 5.0)
        self.declare_parameter('focus_controller', '')
        self.declare_parameter('realtime_logging', False)
        
        self.check_period = self.get_parameter('check_period').value
        self.focus_controller = self.get_parameter('focus_controller').value
        self.realtime_logging = self.get_parameter('realtime_logging').value
        
        # Expected controllers
        if self.focus_controller:
            self.expected_controllers = [self.focus_controller]
        else:
            self.expected_controllers = [
                'scaled_joint_trajectory_controller',
                'joint_state_broadcaster', 
                'force_torque_sensor_broadcaster',
                'ur_admittance_controller'
            ]
        
        # Service client
        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        
        # Data status tracking
        self.data_received = {
            'joint_states': False,
            'ft_sensor': False,
            'admittance_velocity': False
        }
        
        # Subscribers
        self.create_subscription(JointState, '/joint_states', 
            lambda msg: self.update_status('joint_states'), 10)
        self.create_subscription(WrenchStamped, '/ft_sensor_readings',
            lambda msg: self.ft_callback(msg), 10)
        self.create_subscription(Twist, '/ur_admittance_controller/cartesian_velocity_command',
            lambda msg: self.velocity_callback(msg), 10)
        
        # Timer for periodic checks
        self.create_timer(self.check_period, self.check_status)
        
        self.get_logger().info(f"🚀 Status checker started (checking every {self.check_period}s)")
        if self.focus_controller:
            self.get_logger().info(f"🎯 Focusing on: {self.focus_controller}")

    def update_status(self, topic):
        self.data_received[topic] = True

    def ft_callback(self, msg):
        self.data_received['ft_sensor'] = True
        
        if self.realtime_logging:
            force = (msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)**0.5
            if force > 1.0:
                self.get_logger().info(f"🔧 Force detected: {force:.1f}N", throttle_duration_sec=1.0)

    def velocity_callback(self, msg):
        self.data_received['admittance_velocity'] = True
        
        if self.realtime_logging:
            vel = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
            if vel > 0.001:
                self.get_logger().info(f"🎯 Motion: {vel:.3f}m/s", throttle_duration_sec=1.0)

    def check_status(self):
        """Main status check with better error handling"""
        self.get_logger().info("\n========== STATUS CHECK ==========")
        
        # Check controller manager
        if not self.list_controllers_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Controller Manager not available!")
            self.get_logger().info("💡 Start robot/simulation first:")
            self.get_logger().info("   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.100")
            self.get_logger().info("   # OR for simulation:")
            self.get_logger().info("   ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e")
            return
        
        # Get controllers and data status
        controllers_ok = self.check_controllers()
        data_ok = self.check_data_flow()
        
        # Overall status with recommendations
        if controllers_ok and data_ok:
            self.get_logger().info("\n✅ SYSTEM READY")
            self.get_logger().info("🎯 Ready for admittance control operations!")
        elif controllers_ok:
            self.get_logger().warn("\n⚠️  Controllers OK, but missing data")
            self.get_logger().info("💡 Check that robot is connected and force/torque sensor is publishing")
        else:
            self.get_logger().error("\n❌ SYSTEM NOT READY")
            self.get_logger().info("💡 Start controllers and verify robot connection")
        
        # Quick commands
        self.get_logger().info("\n📋 Quick commands:")
        self.get_logger().info("  • ros2 control list_controllers")
        self.get_logger().info("  • ros2 topic echo /ft_sensor_readings --once")
        self.get_logger().info("  • ros2 param set /ur_admittance_controller mass.0 5.0")
        self.get_logger().info("  • ros2 param set /ur_admittance_controller realtime_logging true")

    def check_controllers(self):
        """Check controller status with improved error handling"""
        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        except Exception as e:
            self.get_logger().error(f"Exception while checking controllers: {e}")
            return False
        
        if not future.result():
            self.get_logger().error("Failed to get controller list - service call failed")
            return False
        
        active_controllers = {c.name: c.state for c in future.result().controller}
        
        self.get_logger().info("\n🎮 Controllers:")
        all_ok = True
        
        for controller in self.expected_controllers:
            if controller in active_controllers:
                state = active_controllers[controller]
                if state == "active":
                    self.get_logger().info(f"  ✅ {controller}")
                else:
                    self.get_logger().warn(f"  ⚠️  {controller} [{state}]")
                    if state == "inactive":
                        self.get_logger().info(f"     → ros2 control set_controller_state {controller} configure")
                        self.get_logger().info(f"     → ros2 control set_controller_state {controller} activate")
                    all_ok = False
            else:
                self.get_logger().error(f"  ❌ {controller} [NOT FOUND]")
                all_ok = False
        
        return all_ok

    def check_data_flow(self):
        self.get_logger().info("\n📡 Data flow:")
        all_ok = True
        
        for topic, status in self.data_received.items():
            if status:
                self.get_logger().info(f"  ✅ {topic}")
            else:
                self.get_logger().warn(f"  ❌ {topic}")
                all_ok = False
        
        return all_ok


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = SystemStatusChecker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Status checker stopped by user")
    except Exception as e:
        if node:
            node.get_logger().error(f"Status checker failed: {e}")
        else:
            print(f"Failed to start status checker: {e}")
    finally:
        try:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Cleanup error: {e}")


if __name__ == '__main__':
    main()