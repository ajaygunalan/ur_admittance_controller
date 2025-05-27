#!/usr/bin/env python3
"""
Unified test implementations for UR Admittance Controller.
All test modes are consolidated here using the shared utilities.
"""
import sys
import time
import threading
from typing import Dict, List, Optional
from geometry_msgs.msg import Twist, WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
from ur_admittance_utils import (
    URAdmittanceTestBase, ForceTestMixin, StiffnessControlMixin, 
    SystemMonitorMixin, TestConfig, TestMode
)
class ImpedanceTest(URAdmittanceTestBase, ForceTestMixin, StiffnessControlMixin):
    """Test impedance vs admittance modes"""
    
    def __init__(self, config: TestConfig = TestConfig()):
        super().__init__('impedance_test_node', config)
        
        self.create_publisher('force', WrenchStamped, config.force_topic)
        self.create_service_client('parameters', SetParameters, 
                                  f'/{config.controller_name}/set_parameters')
        
        self.log_info("ImpedanceTest initialized")
        
    def execute(self) -> bool:
        """Execute the impedance test sequence"""
        self.log_info("=== Starting Impedance Mode Test ===")
        
        if not self.wait_for_service('parameters'):
            return False
            
        self.log_info("\n--- Testing Pure Admittance Mode (K=0) ---")
        if not self.set_stiffness_parameters([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
            return False
            
        time.sleep(0.5)
        
        if not self.apply_force_sequence(self.config.test_force, 
                                        self.config.test_duration, 'x'):
            return False
            
        self.log_info("Expected: Robot should STAY at new position after force removal")
        time.sleep(self.config.test_duration + 1.0)
        
        self.log_info("\n--- Testing Impedance Mode (K>0) ---")
        if not self.set_stiffness_parameters([100.0, 100.0, 100.0, 10.0, 10.0, 10.0]):
            return False
            
        time.sleep(0.5)
        
        if not self.apply_force_sequence(self.config.test_force, 
                                        self.config.test_duration, 'x'):
            return False
            
        self.log_info("Expected: Robot should RETURN to original position after force removal")
        time.sleep(self.config.test_duration + 1.0)
        
        self.log_info("\n--- Testing Mixed Mode (XY compliant, Z stiff) ---")
        if not self.set_stiffness_parameters([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]):
            return False
            
        time.sleep(0.5)
        
        if not self.apply_force_sequence(self.config.test_force, 
                                        self.config.test_duration, 'x'):
            return False
        time.sleep(self.config.test_duration + 1.0)
        
        if not self.apply_force_sequence(self.config.test_force, 
                                        self.config.test_duration, 'z'):
            return False
        time.sleep(self.config.test_duration + 1.0)
        
        self.log_info("\n✅ All impedance tests completed successfully!")
        return True
class SafeStartupTest(URAdmittanceTestBase):
    """Test safe startup sequence with pose error monitoring"""
    
    def __init__(self, config: TestConfig = TestConfig()):
        super().__init__('safe_startup_test_node', config)
        
        self.max_error = 0.0
        self.error_samples = []
        self.test_started = False
        
        self.create_subscriber('pose_error', Twist, self._pose_error_callback)
        self.create_service_client('move_to_start', Trigger,
                                  f'/{config.controller_name}/move_to_start_pose')
        
        self.log_info("SafeStartupTest initialized")
        
    def _pose_error_callback(self, msg: Twist):
        """Track pose errors during test"""
        if not self.test_started:
            return
            
        error = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        
        with self._lock:
            self.max_error = max(self.max_error, error)
            self.error_samples.append((self.get_clock().now(), error))
            
            if error > self.config.max_error_threshold * 0.8:
                self.log_warn(f"High error: {error:.4f}m (threshold: {self.config.max_error_threshold}m)")
    
    def execute(self) -> bool:
        """Execute the safe startup test"""
        self.log_info("=== Starting Safe Startup Test ===")
        
        self.max_error = 0.0
        self.error_samples.clear()
        self.test_started = False
        
        if not self.wait_for_service('move_to_start'):
            return False
            
        self.test_started = True
        self.log_info("Started error monitoring")
        
        request = Trigger.Request()
        result = self.call_service_async('move_to_start', request)
        
        if not result or not result.success:
            self.log_error(f"Startup service failed: {result.message if result else 'No response'}")
            return False
            
        self.log_info(f"Startup initiated: {result.message}")
        
        start_time = self.get_clock().now()
        last_update = start_time
        
        while (self.get_clock().now() - start_time).nanoseconds < self.config.test_timeout * 1e9:
            if self._shutdown_event.wait(0.1):
                break
                
            current_time = self.get_clock().now()
            if (current_time - last_update).nanoseconds > self.config.progress_update_rate * 1e9:
                elapsed = (current_time - start_time).nanoseconds / 1e9
                with self._lock:
                    self.log_info(f"Progress: {elapsed:.1f}s, max error: {self.max_error:.4f}m")
                last_update = current_time
                
            with self._lock:
                if self.max_error > self.config.max_error_threshold:
                    self.log_error(f"Error threshold exceeded: {self.max_error:.4f}m")
                    return False
        
        self.test_started = False
        with self._lock:
            num_samples = len(self.error_samples)
            
        self.log_info(f"Test completed with {num_samples} samples")
        
        if self.max_error > self.config.max_error_threshold:
            self.log_error(f"FAILED: Max error {self.max_error:.4f}m > {self.config.max_error_threshold}m")
            return False
        else:
            self.log_info(f"PASSED: Max error {self.max_error:.4f}m < {self.config.max_error_threshold}m")
            return True
class SystemStatusMonitor(URAdmittanceTestBase, SystemMonitorMixin):
    """System status monitoring and diagnostics"""
    
    def __init__(self, config: TestConfig = TestConfig()):
        super().__init__('system_status_monitor', config)
        
        if config.focus_controller:
            self._expected_controllers = [config.focus_controller]
        else:
            self._expected_controllers = [
                'scaled_joint_trajectory_controller',
                'joint_state_broadcaster',
                'force_torque_sensor_broadcaster', 
                'ur_admittance_controller'
            ]
        
        self.create_service_client('list_controllers', ListControllers,
                                  '/controller_manager/list_controllers')
        
        self._data_received = {
            'joint_states': False,
            'ft_sensor': False,
            'admittance_velocity': False
        }
        
        self.create_subscriber('joint_states', JointState,
                              lambda msg: self.update_data_status('joint_states'), 10)
        self.create_subscriber('ft_sensor', WrenchStamped,
                              lambda msg: self._ft_callback(msg), 10)
        self.create_subscriber('velocity_cmd', Twist,
                              lambda msg: self._velocity_callback(msg), 10)
        
        self.log_info("SystemStatusMonitor initialized")
        
    def _ft_callback(self, msg: WrenchStamped):
        """Force/torque data callback"""
        self.update_data_status('ft_sensor')
        
        if self.config.realtime_logging:
            force = (msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)**0.5
            if force > 1.0:
                self.log_info(f"🔧 Force detected: {force:.1f}N")
                
    def _velocity_callback(self, msg: Twist):
        """Velocity command callback"""
        self.update_data_status('admittance_velocity')
        
        if self.config.realtime_logging:
            vel = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
            if vel > 0.001:
                self.log_info(f"🎯 Motion: {vel:.3f}m/s")
    
    def print_status_report(self):
        """Print formatted status report"""
        self.log_info("\n========== STATUS CHECK ==========")
        
        if not self.wait_for_service('list_controllers', timeout=2.0):
            self.log_error("❌ Controller Manager not available!")
            self.log_info("💡 Start robot/simulation first")
            return False
            
        controller_states = self.check_controllers()
        data_status = self.check_data_flow()
        
        self.log_info("\n🎮 Controllers:")
        all_controllers_ok = True
        
        for controller in self._expected_controllers:
            if controller in controller_states:
                state = controller_states[controller]
                if state == "active":
                    self.log_info(f"  ✅ {controller}")
                else:
                    self.log_warn(f"  ⚠️  {controller} [{state}]")
                    all_controllers_ok = False
            else:
                self.log_error(f"  ❌ {controller} [NOT FOUND]")
                all_controllers_ok = False
                
        self.log_info("\n📡 Data flow:")
        all_data_ok = True
        
        for topic, status in data_status.items():
            if status:
                self.log_info(f"  ✅ {topic}")
            else:
                self.log_warn(f"  ❌ {topic}")
                all_data_ok = False
                
        if all_controllers_ok and all_data_ok:
            self.log_info("\n✅ SYSTEM READY")
        else:
            self.log_error("\n❌ SYSTEM NOT READY")
            
        self.log_info("\n📋 Quick commands:")
        self.log_info("  • ros2 control list_controllers")
        self.log_info("  • ros2 topic echo /ft_sensor_readings --once")
        self.log_info("  • ros2 param set /ur_admittance_controller admittance.mass [5.0,5.0,5.0,0.5,0.5,0.5]")
        
        return all_controllers_ok and all_data_ok
    
    def execute(self) -> bool:
        """Execute system monitoring"""
        if self.config.check_period <= 0:
            return self.print_status_report()
        else:
            self.log_info(f"Starting continuous monitoring (period: {self.config.check_period}s)")
            
            while not self._shutdown_event.wait(self.config.check_period):
                self.print_status_report()
                
            return True
def main():
    """Main entry point with command-line interface"""
    import argparse
    
    parser = argparse.ArgumentParser(description="UR Admittance Controller Test Suite")
    parser.add_argument('mode', choices=['impedance', 'safe_startup', 'status', 'monitor'],
                       help="Test mode to execute")
    parser.add_argument('--controller', default='ur_admittance_controller',
                       help="Controller name")
    parser.add_argument('--force', type=float, default=20.0,
                       help="Test force magnitude (N)")
    parser.add_argument('--duration', type=float, default=2.0,
                       help="Test duration (s)")
    parser.add_argument('--error-threshold', type=float, default=0.15,
                       help="Max pose error threshold (m)")
    parser.add_argument('--check-period', type=float, default=5.0,
                       help="Status check period (0 for single check)")
    parser.add_argument('--realtime-log', action='store_true',
                       help="Enable real-time logging")
    
    args = parser.parse_args()
    
    config = TestConfig(
        controller_name=args.controller,
        test_force=args.force,
        test_duration=args.duration,
        max_error_threshold=args.error_threshold,
        check_period=args.check_period if args.mode == 'monitor' else 0,
        realtime_logging=args.realtime_log
    )
    
    mode_map = {
        'impedance': TestMode.IMPEDANCE,
        'safe_startup': TestMode.SAFE_STARTUP,
        'status': TestMode.SYSTEM_STATUS,
        'monitor': TestMode.CONTINUOUS_MONITOR
    }
    
    from ur_admittance_utils import TestRunner
    runner = TestRunner(config)
    return runner.run(mode_map[args.mode])
if __name__ == '__main__':
    sys.exit(main())