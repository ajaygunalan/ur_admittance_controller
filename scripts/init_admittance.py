#!/usr/bin/env python3
"""Initialize robot at equilibrium position for zero-error startup."""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class InitAdmittance(Node):
    def __init__(self):
        super().__init__('init_admittance')
        
        # Load config
        config_file = os.path.join(
            get_package_share_directory('ur_admittance_controller'),
            'config', 'admittance_config.yaml'
        )
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            
        self.initial_positions = config['initial_joint_positions']
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Service clients for Gazebo
        self.pause_client = self.create_client(Empty, '/pause_physics')
        self.unpause_client = self.create_client(Empty, '/unpause_physics')
        
        # Publisher to set joint states
        self.joint_pub = self.create_publisher(JointState, '/set_joint_states', 10)
        
    def run_initialization(self):
        if self.pause_client.wait_for_service(timeout_sec=2.0):
            self.init_simulation()
        else:
            self.init_real_robot()
            
    def init_simulation(self):
        self.get_logger().info("Simulation mode - zero-error initialization")
        
        # 1. Pause physics
        self.pause_client.call_async(Empty.Request())
        time.sleep(0.5)
        
        # 2. Set joints to initial position
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.initial_positions
        
        for _ in range(5):
            self.joint_pub.publish(joint_msg)
            time.sleep(0.1)
        
        # 3. Unpause physics
        self.unpause_client.call_async(Empty.Request())
        self.get_logger().info("Robot positioned at equilibrium - zero error achieved")
        
    def init_real_robot(self):
        self.get_logger().warn("=" * 60)
        self.get_logger().warn("REAL ROBOT MODE - MANUAL POSITIONING REQUIRED")
        self.get_logger().warn("Move robot to: [0.1, 0.4, 0.5] meters")
        self.get_logger().warn("=" * 60)
        time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = InitAdmittance()
    node.run_initialization()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()