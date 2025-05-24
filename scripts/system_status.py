#!/usr/bin/env python3
"""
Complete UR Admittance Control System Status Checker
Monitors the complete system including all controllers, sensors, and data flow

Usage:
  ros2 run ur_admittance_controller system_status.py
  ros2 run ur_admittance_controller system_status.py --ros-args -p use_sim:=false -p ur_type:=ur10e
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
import time
from collections import defaultdict


class SystemStatusChecker(Node):
    def __init__(self):
        super().__init__('system_status_checker')
        
        # Parameters
        self.declare_parameter('use_sim', True)
        self.declare_parameter('ur_type', 'ur5e')
        self.declare_parameter('check_period', 10.0)
        self.declare_parameter('expected_controllers', [
            'scaled_joint_trajectory_controller',
            'joint_state_broadcaster', 
            'force_torque_sensor_broadcaster',
            'ur_admittance_controller'
        ])
        
        self.use_sim = self.get_parameter('use_sim').value
        self.ur_type = self.get_parameter('ur_type').value
        self.check_period = self.get_parameter('check_period').value
        self.expected_controllers = self.get_parameter('expected_controllers').value
        
        # Service clients
        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        
        # Data monitoring subscribers
        self.subscribers = {}
        self.data_status = defaultdict(bool)
        self.data_counters = defaultdict(int)
        self.last_values = {}
        
        # Subscribe to all important topics
        self.setup_subscribers()
        
        # Timer for periodic system checks
        self.status_timer = self.create_timer(self.check_period, self.check_system_status)
        
        # Initial system info
        self.get_logger().info("🚀 UR Admittance Control System Monitor Started")
        self.get_logger().info(f"🤖 Robot: {self.ur_type.upper()}")
        self.get_logger().info(f"🌍 Mode: {'Gazebo Simulation' if self.use_sim else 'Real Robot'}")
        self.get_logger().info(f"📊 Monitoring {len(self.expected_controllers)} controllers")

    def setup_subscribers(self):
        """Set up subscribers for all important system topics"""
        
        # Joint states
        self.subscribers['joint_states'] = self.create_subscription(
            JointState, '/joint_states', 
            lambda msg: self.data_callback('joint_states', msg), 10
        )
        
        # F/T sensor
        self.subscribers['ft_sensor'] = self.create_subscription(
            WrenchStamped, '/ft_sensor_readings',
            lambda msg: self.data_callback('ft_sensor', msg), 10
        )
        
        # Admittance controller output
        self.subscribers['admittance_velocity'] = self.create_subscription(
            Twist, '/ur_admittance_controller/cartesian_velocity_command',
            lambda msg: self.data_callback('admittance_velocity', msg), 10
        )
        
        # Trajectory commands monitoring
        self.subscribers['trajectory_commands'] = self.create_subscription(
            JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory',
            lambda msg: self.data_callback('trajectory_commands', msg), 10
        )
        
        # Speed scaling (for real robot)
        if not self.use_sim:
            self.subscribers['speed_scaling'] = self.create_subscription(
                Float64, '/speed_scaling_factor',
                lambda msg: self.data_callback('speed_scaling', msg), 10
            )
        
        # Robot status topics (for real robot)
        if not self.use_sim:
            self.subscribers['robot_mode'] = self.create_subscription(
                Float64, '/robot_mode',
                lambda msg: self.data_callback('robot_mode', msg), 10
            )

    def data_callback(self, topic_name, msg):
        """Generic callback for data monitoring"""
        self.data_status[topic_name] = True
        self.data_counters[topic_name] += 1
        
        # Store last values for analysis
        if topic_name == 'ft_sensor':
            force_mag = (msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)**0.5
            torque_mag = (msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.torque.z**2)**0.5
            self.last_values['ft_force'] = force_mag
            self.last_values['ft_torque'] = torque_mag
            
        elif topic_name == 'admittance_velocity':
            vel_mag = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
            ang_mag = (msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)**0.5
            self.last_values['admittance_linear'] = vel_mag
            self.last_values['admittance_angular'] = ang_mag
            
        elif topic_name == 'joint_states':
            self.last_values['joint_count'] = len(msg.position)
            
        elif topic_name == 'speed_scaling':
            self.last_values['speed_scaling'] = msg.data
            
        elif topic_name == 'robot_mode':
            self.last_values['robot_mode'] = msg.data

    def check_controller_spawning(self):
        """Check if controllers are being spawned"""
        # Check for spawner processes
        import subprocess
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        if 'spawner' in result.stdout:
            self.get_logger().info("⏳ Controller spawner active - wait for completion")
            return True
        return False

    def check_system_status(self):
        """Comprehensive system status check"""
        
        self.get_logger().info("\n\n==============================================")
        self.get_logger().info("🔍 SYSTEM STATUS CHECK - " + time.strftime("%H:%M:%S"))
        
        # Check if controllers are still being spawned
        spawning = self.check_controller_spawning()
        if spawning:
            self.get_logger().info("⏳ System initialization in progress...")
            return
        
        self.get_logger().info("=" * 80)
        
        # 1. Check Controller Manager availability
        if not self.check_controller_manager():
            return
        
        # 2. Check all controllers
        controller_status = self.check_all_controllers()
        
        # 3. Check data flow
        data_flow_status = self.check_data_flow()
        
        # 4. Check system integration
        integration_status = self.check_system_integration()
        
        # 5. Overall system health
        overall_health = self.assess_overall_health(controller_status, data_flow_status, integration_status)
        
        # 6. Provide guidance
        self.provide_system_guidance(overall_health)
        
        self.get_logger().info("=" * 80)

    def check_controller_manager(self):
        """Check if controller manager is available"""
        
        if not self.list_controllers_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ CRITICAL: Controller Manager not available!")
            self.get_logger().error("   Check if robot/simulation is running")
            return False
        
        self.get_logger().info("✅ Controller Manager: Available")
        return True

    def check_all_controllers(self):
        """Check status of all expected controllers"""
        
        # Get controller list
        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result() is None:
            self.get_logger().error("❌ Failed to get controller list")
            return False
        
        controllers = future.result().controller
        controller_dict = {c.name: c.state for c in controllers}
        
        self.get_logger().info("🎮 CONTROLLER STATUS:")
        
        all_controllers_ok = True
        critical_controllers = ['scaled_joint_trajectory_controller', 'joint_state_broadcaster']
        admittance_controllers = ['force_torque_sensor_broadcaster', 'ur_admittance_controller']
        
        for controller_name in self.expected_controllers:
            if controller_name in controller_dict:
                state = controller_dict[controller_name]
                
                if state == "active":
                    icon = "✅"
                elif state == "inactive":
                    icon = "⏸️ "
                else:
                    icon = "❌"
                    if controller_name in critical_controllers:
                        all_controllers_ok = False
                
                self.get_logger().info(f"   {icon} {controller_name}: {state}")
                
                # Special handling for admittance controllers
                if controller_name in admittance_controllers and state != "active":
                    self.get_logger().warn(f"      💡 Admittance control requires {controller_name} to be active")
                    
            else:
                self.get_logger().error(f"   ❌ {controller_name}: NOT FOUND")
                if controller_name in critical_controllers:
                    all_controllers_ok = False
        
        # Show additional controllers
        additional_controllers = [c for c in controllers if c.name not in self.expected_controllers]
        if additional_controllers:
            self.get_logger().info("   📋 Additional Controllers:")
            for controller in additional_controllers:
                icon = "✅" if controller.state == "active" else "⏸️ "
                self.get_logger().info(f"      {icon} {controller.name}: {controller.state}")
        
        return all_controllers_ok

    def check_data_flow(self):
        """Check data flow through the system"""
        
        self.get_logger().info("📡 DATA FLOW STATUS:")
        
        expected_topics = ['joint_states', 'ft_sensor', 'admittance_velocity']
        if not self.use_sim:
            expected_topics.extend(['speed_scaling', 'robot_mode'])
        
        all_data_ok = True
        
        for topic in expected_topics:
            if self.data_status[topic]:
                count = self.data_counters[topic]
                icon = "✅"
                
                # Check data quality
                if topic == 'ft_sensor' and 'ft_force' in self.last_values:
                    force = self.last_values['ft_force']
                    detail = f"(Force: {force:.1f}N)"
                elif topic == 'admittance_velocity' and 'admittance_linear' in self.last_values:
                    vel = self.last_values['admittance_linear']
                    detail = f"(Velocity: {vel:.3f}m/s)"
                elif topic == 'joint_states' and 'joint_count' in self.last_values:
                    joints = self.last_values['joint_count']
                    detail = f"({joints} joints)"
                else:
                    detail = f"({count} msgs)"
                
                self.get_logger().info(f"   {icon} {topic}: Active {detail}")
            else:
                icon = "❌"
                all_data_ok = False
                self.get_logger().info(f"   {icon} {topic}: No data")
        
        return all_data_ok

    def check_system_integration(self):
        """Check integration between system components"""
        
        self.get_logger().info("🔗 SYSTEM INTEGRATION:")
        
        integration_ok = True
        
        # Check F/T to admittance integration
        if self.data_status['ft_sensor'] and self.data_status['admittance_velocity']:
            ft_active = self.last_values.get('ft_force', 0) > 1.0
            admittance_active = self.last_values.get('admittance_linear', 0) > 0.001
            
            if ft_active and admittance_active:
                self.get_logger().info("   ✅ F/T → Admittance: Active response detected")
            elif ft_active and not admittance_active:
                self.get_logger().warn("   ⚠️  F/T sensor active but no admittance response")
                integration_ok = False
            else:
                self.get_logger().info("   ℹ️  F/T → Admittance: Standby (no force applied)")
        else:
            self.get_logger().error("   ❌ F/T → Admittance: Data missing")
            integration_ok = False
        
        # Check real robot specific integration
        if not self.use_sim:
            if 'speed_scaling' in self.last_values:
                scaling = self.last_values['speed_scaling']
                if scaling < 0.1:
                    self.get_logger().warn(f"   ⚠️  Speed scaling very low: {scaling:.1%}")
                else:
                    self.get_logger().info(f"   ✅ Speed scaling: {scaling:.1%}")
            
            if 'robot_mode' in self.last_values:
                mode = self.last_values['robot_mode']
                if mode == 7:  # Running mode
                    self.get_logger().info("   ✅ Robot mode: Running")
                else:
                    self.get_logger().warn(f"   ⚠️  Robot mode: {mode} (not running)")
                    integration_ok = False
        
        return integration_ok

    def assess_overall_health(self, controllers_ok, data_ok, integration_ok):
        """Assess overall system health"""
        
        self.get_logger().info("💊 OVERALL SYSTEM HEALTH:")
        
        if controllers_ok and data_ok and integration_ok:
            health = "EXCELLENT"
            icon = "🟢"
            self.get_logger().info(f"   {icon} Status: {health}")
            self.get_logger().info("   🎉 All systems operational!")
            return "excellent"
            
        elif controllers_ok and data_ok:
            health = "GOOD"
            icon = "🟡"
            self.get_logger().info(f"   {icon} Status: {health}")
            self.get_logger().info("   ⚠️  Minor integration issues detected")
            return "good"
            
        elif controllers_ok:
            health = "DEGRADED"
            icon = "🟠"
            self.get_logger().info(f"   {icon} Status: {health}")
            self.get_logger().info("   ❌ Data flow issues detected")
            return "degraded"
            
        else:
            health = "CRITICAL"
            icon = "🔴"
            self.get_logger().info(f"   {icon} Status: {health}")
            self.get_logger().info("   ❌ Critical controller issues detected")
            return "critical"

    def provide_system_guidance(self, health_status):
        """Provide troubleshooting guidance based on system status"""
        
        self.get_logger().info("💡 GUIDANCE & NEXT STEPS:")
        
        if health_status == "excellent":
            self.get_logger().info("   🧪 Ready for testing:")
            if self.use_sim:
                self.get_logger().info("      1. In Gazebo, enable Force mode (F key)")
                self.get_logger().info("      2. Click and drag robot end-effector") 
                self.get_logger().info("      3. Robot should move compliantly")
            else:
                self.get_logger().info("      1. Gently apply force to robot end-effector")
                self.get_logger().info("      2. Robot should move compliantly")
                self.get_logger().info("      3. Robot should stop when force removed")
                
        elif health_status == "good":
            self.get_logger().info("   🔧 System mostly working - check:")
            self.get_logger().info("      • F/T sensor calibration")
            self.get_logger().info("      • Admittance parameters")
            self.get_logger().info("      • Network latency (real robot)")
            
        elif health_status == "degraded":
            self.get_logger().info("   🚨 Data issues - check:")
            self.get_logger().info("      • ros2 topic list | grep ft")
            self.get_logger().info("      • ros2 topic hz /joint_states")
            self.get_logger().info("      • Controller parameter files")
            
        elif health_status == "critical":
            self.get_logger().info("   🆘 Critical issues - restart sequence:")
            self.get_logger().info("      1. Stop all controllers")
            self.get_logger().info("      2. Check robot/simulation connection")
            self.get_logger().info("      3. Restart controller manager")
            self.get_logger().info("      4. Re-launch system")
        
        # Always provide useful commands
        self.get_logger().info("   📋 Useful commands:")
        self.get_logger().info("      ros2 control list_controllers")
        self.get_logger().info("      ros2 topic echo /ft_sensor_readings --once")
        self.get_logger().info("      ros2 param set /ur_admittance_controller mass.0 5.0")

    def get_current_time(self):
        """Get formatted current time"""
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SystemStatusChecker()
        node.get_logger().info("🚀 System status checker ready")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info("👋 System status checker shutting down")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()