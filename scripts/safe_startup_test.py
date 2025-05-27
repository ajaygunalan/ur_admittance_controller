#!/usr/bin/env python3
"""
Safe startup test for UR Admittance Controller.
Tests that the robot can safely move to start pose without exceeding error thresholds.
"""

import rclpy
from rclpy.node import Node
import time
import sys
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import MultiThreadedExecutor
import threading

class SafeStartupTest(Node):
    def __init__(self):
        super().__init__('safe_startup_test')
        
        self.declare_parameter('max_error_threshold', 0.15)
        self.declare_parameter('test_timeout', 15.0)
        self.declare_parameter('service_timeout', 5.0)
        self.declare_parameter('error_topic', '/ur_admittance_controller/pose_error')
        self.declare_parameter('service_name', '/ur_admittance_controller/move_to_start_pose')
        
        self.max_error_threshold = self.get_parameter('max_error_threshold').get_parameter_value().double_value
        self.test_timeout = self.get_parameter('test_timeout').get_parameter_value().double_value
        self.service_timeout = self.get_parameter('service_timeout').get_parameter_value().double_value
        error_topic = self.get_parameter('error_topic').get_parameter_value().string_value
        service_name = self.get_parameter('service_name').get_parameter_value().string_value
        
        self.max_error = 0.0
        self.error_samples = []
        self.test_started = False
        self.service_completed = False
        self.lock = threading.Lock()
        
        self.pose_error_sub = self.create_subscription(
            Twist, error_topic, self.pose_error_callback, 10)
            
        self.service_client = self.create_client(Trigger, service_name)
        
        self.get_logger().info(f"Safe Startup Test initialized. Error threshold: {self.max_error_threshold}m")
    def pose_error_callback(self, msg):
        """Track pose errors during the test"""
        if not self.test_started:
            return
            
        error = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        
        with self.lock:
            self.max_error = max(self.max_error, error)
            self.error_samples.append((self.get_clock().now(), error))
            
            if error > self.max_error_threshold * 0.8:
                self.get_logger().warn(f"High error detected: {error:.4f}m (threshold: {self.max_error_threshold}m)")
            else:
                self.get_logger().debug(f"Current error: {error:.4f}m, Max error: {self.max_error:.4f}m")
    
    def reset_test_state(self):
        """Reset test state before starting a new test"""
        with self.lock:
            self.max_error = 0.0
            self.error_samples.clear()
            self.test_started = False
            self.service_completed = False
    
    def wait_for_service_ready(self):
        """Wait for the move_to_start_pose service to be available"""
        self.get_logger().info("Waiting for move_to_start_pose service...")
        
        if not self.service_client.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().error(f"Service not available after {self.service_timeout}s timeout")
            return False
            
        self.get_logger().info("Service is ready")
        return True
    
    def call_move_to_start_pose(self):
        """Call the move_to_start_pose service"""
        request = Trigger.Request()
        
        try:
            future = self.service_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)
            
            if not future.done():
                self.get_logger().error("Service call timed out")
                return False
                
            result = future.result()
            if result.success:
                self.get_logger().info(f"Service call succeeded: {result.message}")
                return True
            else:
                self.get_logger().error(f"Service call failed: {result.message}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Exception during service call: {str(e)}")
            return False
    def test_startup(self):
        """Execute the safe startup test"""
        self.get_logger().info("=== Starting Safe Startup Test ===")
        
        self.reset_test_state()
        
        if not self.wait_for_service_ready():
            return False
        
        self.test_started = True
        self.get_logger().info("Started error monitoring")
        
        if not self.call_move_to_start_pose():
            return False
        
        self.service_completed = True
        
        self.get_logger().info(f"Waiting up to {self.test_timeout}s for startup sequence to complete...")
        
        start_time = self.get_clock().now()
        last_progress_time = start_time
        
        while (self.get_clock().now() - start_time).nanoseconds < self.test_timeout * 1e9:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            current_time = self.get_clock().now()
            if (current_time - last_progress_time).nanoseconds > 2e9:
                elapsed = (current_time - start_time).nanoseconds / 1e9
                with self.lock:
                    self.get_logger().info(f"Progress: {elapsed:.1f}s elapsed, max error so far: {self.max_error:.4f}m")
                last_progress_time = current_time
            
            with self.lock:
                if self.max_error > self.max_error_threshold:
                    self.get_logger().error(f"Test failed early: error {self.max_error:.4f}m > threshold {self.max_error_threshold}m")
                    return False
        
        with self.lock:
            final_max_error = self.max_error
            num_samples = len(self.error_samples)
        
        self.get_logger().info(f"Test completed. Processed {num_samples} error samples")
        
        if final_max_error > self.max_error_threshold:
            self.get_logger().error(f"FAILED: Max error {final_max_error:.4f}m > {self.max_error_threshold}m")
            return False
        else:
            self.get_logger().info(f"PASSED: Max error {final_max_error:.4f}m < {self.max_error_threshold}m")
            return True


def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    test_node = SafeStartupTest()
    executor.add_node(test_node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        result = test_node.test_startup()
        
        if result:
            print("\n✅ TEST PASSED: Safe startup sequence completed successfully!")
            exit_code = 0
        else:
            print("\n❌ TEST FAILED: Safe startup sequence exceeded error limits!")
            exit_code = 1
    
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
        exit_code = 130
    except Exception as e:
        test_node.get_logger().error(f"Test failed with exception: {str(e)}")
        exit_code = 2
    finally:
        test_node.get_logger().info("Cleaning up...")
        executor.shutdown()
        test_node.destroy_node()
        rclpy.shutdown()
        
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
test_runner.launch.py