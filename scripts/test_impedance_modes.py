#!/usr/bin/env python3
"""
Test script for impedance vs admittance modes in UR Admittance Controller.
This script demonstrates the difference between pure admittance (K=0) and impedance (K>0) control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
import sys
import threading

class ImpedanceTester(Node):
    def __init__(self):
        super().__init__('impedance_tester')
        
        # Parameters
        self.declare_parameter('force_topic', '/ft_sensor_readings')
        self.declare_parameter('test_force', 20.0)
        self.declare_parameter('test_duration', 2.0)
        
        topic = self.get_parameter('force_topic').get_parameter_value().string_value
        self.test_force = self.get_parameter('test_force').get_parameter_value().double_value
        self.test_duration = self.get_parameter('test_duration').get_parameter_value().double_value
        
        self.publisher = self.create_publisher(WrenchStamped, topic, 10)
        
        # Service client for parameter setting
        self.param_client = self.create_client(SetParameters, '/ur_admittance_controller/set_parameters')
        
        self.get_logger().info("ImpedanceTester initialized")
        
    def create_wrench_msg(self, force_x=0.0):
        """Create a properly timestamped WrenchStamped message"""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.wrench.force.x = force_x
        return msg
        
    def wait_for_publisher(self, timeout_sec=5.0):
        """Wait for publisher to have subscribers"""
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < timeout_sec * 1e9:
            if self.publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().warn("No subscribers found for force topic")
        return False
                
    def set_stiffness_parameters(self, stiffness_values):
        """Set stiffness parameters via service call"""
        if not self.param_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Parameter service not available!")
            return False
            
        # Create parameter
        param = Parameter()
        param.name = 'admittance.stiffness'
        param.value = ParameterValue(type=7, double_array_value=stiffness_values)  # Type 7 = double array
        
        # Create request
        request = SetParameters.Request()
        request.parameters = [param]
        
        # Call service
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result() and future.result().results[0].successful:
            self.get_logger().info(f"Stiffness set to: {stiffness_values}")
            return True
        else:
            error_msg = "Unknown error"
            if future.result():
                error_msg = future.result().results[0].reason
            self.get_logger().error(f"Failed to set stiffness parameters: {error_msg}")
            return False

    def apply_force_sequence(self, test_name):
        """Apply force, wait, then remove force"""
        self.get_logger().info(f"Starting {test_name}")
        
        # Wait for subscribers
        if not self.wait_for_publisher():
            return False
        
        # Apply force
        msg = self.create_wrench_msg(self.test_force)
        self.publisher.publish(msg)
        self.get_logger().info(f"Applied {self.test_force}N force in X direction")
        
        # Wait for effect (non-blocking)
        def remove_force():
            import time
            time.sleep(self.test_duration)
            msg_zero = self.create_wrench_msg(0.0)
            self.publisher.publish(msg_zero)
            self.get_logger().info("Force removed")
            
        # Run in separate thread to avoid blocking
        thread = threading.Thread(target=remove_force)
        thread.daemon = True
        thread.start()
        
        return True
        
    def test_pure_admittance(self):
        """Test pure admittance mode (K=0) - robot should stay at new position"""
        self.get_logger().info("=== Testing Pure Admittance Mode (K=0) ===")
        
        # Set zero stiffness
        if not self.set_stiffness_parameters([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
            return False
            
        if not self.apply_force_sequence("pure admittance test"):
            return False
            
        self.get_logger().info("Expected behavior: Robot should STAY at new position after force removal")
        return True
        
    def test_impedance(self):
        """Test impedance mode (K>0) - robot should return to original position"""
        self.get_logger().info("=== Testing Impedance Mode (K>0) ===")
        
        # Set non-zero stiffness
        if not self.set_stiffness_parameters([100.0, 100.0, 100.0, 0.0, 0.0, 0.0]):
            return False
            
        if not self.apply_force_sequence("impedance test"):
            return False
            
        self.get_logger().info("Expected behavior: Robot should RETURN to original position after force removal")
        return True

def main():
    rclpy.init()
    tester = ImpedanceTester()
    
    try:
        # Interactive test sequence
        input("Press Enter to test pure admittance mode (K=0)...")
        if not tester.test_pure_admittance():
            sys.exit(1)
        
        input("\nPress Enter to test impedance mode (K>0)...")
        if not tester.test_impedance():
            sys.exit(1)
        
        print("\n✅ All tests completed successfully!")
        
        # Keep node alive for a bit to allow final messages
        import time
        time.sleep(3)
        
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    except Exception as e:
        tester.get_logger().error(f"Test failed with exception: {str(e)}")
        sys.exit(1)
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

