#!/usr/bin/env python3
"""
Consolidated UR Admittance Controller Test Suite
Replaces: ur_admittance_tests.py + ur_admittance_utils.py + system_status.py
"""
import rclpy
import sys
import time
import argparse
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Twist
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

def test_impedance():
    """Test impedance vs admittance modes"""
    rclpy.init()
    node = Node('impedance_test')
    
    # Create publishers and service clients
    force_pub = node.create_publisher(WrenchStamped, '/wrist_ft_sensor', 10)
    param_client = node.create_client(SetParameters, '/ur_admittance_controller/set_parameters')
    
    print("=== Testing Impedance Modes ===")
    
    # Test admittance mode (K=0)
    param = Parameter(name='admittance.stiffness', value=ParameterValue(type=7, double_array_value=[0.0]*6))
    param_client.call_async(SetParameters.Request(parameters=[param]))
    time.sleep(1.0)
    
    # Apply test force
    msg = WrenchStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.wrench.force.x = 20.0
    force_pub.publish(msg)
    print("Applied 20N force in X direction - robot should move and stay")
    time.sleep(3.0)
    
    # Remove force
    msg.wrench.force.x = 0.0
    force_pub.publish(msg)
    time.sleep(2.0)
    
    print("✅ Impedance test completed")
    node.destroy_node()
    rclpy.shutdown()

def test_startup():
    """Monitor safe startup sequence"""
    rclpy.init()
    node = Node('startup_test')
    
    max_error = 0.0
    def pose_error_callback(msg):
        nonlocal max_error
        error = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        max_error = max(max_error, error)
    
    node.create_subscription(Twist, '/admittance_pose_error', pose_error_callback, 10)
    
    print("=== Monitoring Startup Safety ===")
    start_time = time.time()
    while time.time() - start_time < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print(f"Maximum pose error during startup: {max_error:.4f}m")
    print("✅ Startup monitoring completed")
    node.destroy_node()
    rclpy.shutdown()

def test_status():
    """Check system status"""
    rclpy.init()
    node = Node('status_test')
    
    # Check controllers
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if client.wait_for_service(timeout_sec=5.0):
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        
        print("=== Controller Status ===")
        for controller in future.result().controller:
            status = "✅" if controller.state == "active" else "❌"
            print(f"{status} {controller.name}: {controller.state}")
    else:
        print("❌ Controller manager not available")
    
    print("✅ Status check completed")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="UR Admittance Test Suite")
    parser.add_argument('test', choices=['impedance', 'startup', 'status'], help="Test to run")
    args = parser.parse_args()
    
    if args.test == 'impedance':
        test_impedance()
    elif args.test == 'startup':
        test_startup()
    elif args.test == 'status':
        test_status()