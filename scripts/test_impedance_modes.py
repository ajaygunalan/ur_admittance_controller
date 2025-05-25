#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import time

class ImpedanceTester(Node):
    def __init__(self):
        super().__init__('impedance_tester')
        self.publisher = self.create_publisher(WrenchStamped, '/ft_sensor_readings', 10)
        
    def test_pure_admittance(self):
        self.get_logger().info("Testing pure admittance (K=0)")
        msg = WrenchStamped()
        msg.wrench.force.x = 20.0
        self.publisher.publish(msg)
        time.sleep(0.5)
        msg.wrench.force.x = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Force removed - robot should STAY at new position")
        
    def test_impedance(self):
        self.get_logger().info("Testing impedance mode (K>0)")
        msg = WrenchStamped()
        msg.wrench.force.x = 20.0
        self.publisher.publish(msg)
        time.sleep(0.5)
        msg.wrench.force.x = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Force removed - robot should RETURN to original position")

def main():
    rclpy.init()
    tester = ImpedanceTester()
    
    # Test sequence
    input("Press Enter to test pure admittance mode...")
    tester.test_pure_admittance()
    
    input("\nNow set stiffness: ros2 param set /ur_admittance_controller admittance.stiffness [100,100,100,0,0,0]")
    input("Press Enter to test impedance mode...")
    tester.test_impedance()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
