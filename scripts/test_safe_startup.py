#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import sys
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Twist

class SafeStartupTest(Node):
    def __init__(self):
        super().__init__('safe_startup_test')
        self.pose_error_sub = self.create_subscription(
            Twist, '/ur_admittance_controller/pose_error',
            self.pose_error_callback, 10)
        self.max_error = 0.0
        self.get_logger().info("Safe Startup Test initialized. Monitoring pose errors...")
        
    def pose_error_callback(self, msg):
        error = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        self.max_error = max(self.max_error, error)
        self.get_logger().debug(f"Current error: {error:.4f}m, Max error: {self.max_error:.4f}m")
        
    def test_startup(self):
        # Test sequence
        self.get_logger().info("Testing safe startup...")
        
        # Reset max error before starting the test
        self.max_error = 0.0
        
        # Create client for the move_to_start_pose service
        client = self.create_client(Trigger, '/ur_admittance_controller/move_to_start_pose')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            
        # Call the service to start the safe movement
        self.get_logger().info("Calling move_to_start_pose service...")
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Wait for the service call to complete
        rclpy.spin_until_future_complete(self, future)
        
        # Check if the service call was successful
        if future.result().success:
            self.get_logger().info(f"Service call succeeded: {future.result().message}")
        else:
            self.get_logger().error(f"Service call failed: {future.result().message}")
            return False
            
        # Wait for the full startup sequence to complete
        self.get_logger().info("Waiting for startup sequence to complete...")
        time.sleep(10)  # Wait for full startup
        
        # Evaluate test results
        if self.max_error > 0.15:
            self.get_logger().error(f"FAILED: Max error {self.max_error:.4f}m > 0.15m")
            return False
        else:
            self.get_logger().info(f"PASSED: Max error {self.max_error:.4f}m < 0.15m")
            return True


def main(args=None):
    rclpy.init(args=args)
    
    # Create the test node
    test_node = SafeStartupTest()
    
    try:
        # Run the test
        result = test_node.test_startup()
        
        # Exit with appropriate status code
        if result:
            print("\n✅ TEST PASSED: Safe startup sequence completed successfully!")
            exit_code = 0
        else:
            print("\n❌ TEST FAILED: Safe startup sequence exceeded error limits!")
            exit_code = 1
    
    except Exception as e:
        test_node.get_logger().error(f"Test failed with exception: {str(e)}")
        exit_code = 2
    finally:
        # Cleanup
        test_node.destroy_node()
        rclpy.shutdown()
        
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
