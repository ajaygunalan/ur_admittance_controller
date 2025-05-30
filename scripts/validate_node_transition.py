#!/usr/bin/env python3
"""
Validation script for comparing outputs between the original controller
and the new standalone node implementation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import numpy as np
import time

class NodeValidation(Node):
    def __init__(self):
        super().__init__('admittance_node_validation')
        
        # Subscribers for node outputs
        self.cart_vel_sub = self.create_subscription(
            Twist,
            '/admittance_cartesian_velocity',
            self.cartesian_velocity_callback,
            10
        )
        
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Publisher for test forces
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            '/wrist_ft_sensor',
            10
        )
        
        # Storage for comparison
        self.last_cart_vel = None
        self.last_trajectory = None
        
        # Test timer
        self.test_timer = self.create_timer(5.0, self.run_test_sequence)
        self.test_phase = 0
        
        self.get_logger().info('Validation node started')
    
    def cartesian_velocity_callback(self, msg):
        self.last_cart_vel = msg
        self.get_logger().debug(f'Received Cartesian velocity: linear.z={msg.linear.z:.3f}')
    
    def trajectory_callback(self, msg):
        self.last_trajectory = msg
        if msg.points:
            self.get_logger().debug(f'Received trajectory with {len(msg.points)} points')
    
    def run_test_sequence(self):
        """Run through different test scenarios"""
        test_forces = [
            # Phase 0: Zero force (should be in deadband)
            {'force': [0.0, 0.0, 0.0], 'torque': [0.0, 0.0, 0.0], 'description': 'Zero force (deadband test)'},
            # Phase 1: Small Z force
            {'force': [0.0, 0.0, 5.0], 'torque': [0.0, 0.0, 0.0], 'description': 'Small Z force'},
            # Phase 2: Larger Z force
            {'force': [0.0, 0.0, 20.0], 'torque': [0.0, 0.0, 0.0], 'description': 'Larger Z force'},
            # Phase 3: Combined force and torque
            {'force': [5.0, 0.0, 10.0], 'torque': [0.0, 0.0, 2.0], 'description': 'Combined force/torque'},
            # Phase 4: Return to zero
            {'force': [0.0, 0.0, 0.0], 'torque': [0.0, 0.0, 0.0], 'description': 'Return to zero'},
        ]
        
        if self.test_phase < len(test_forces):
            test = test_forces[self.test_phase]
            self.apply_test_force(test['force'], test['torque'])
            self.get_logger().info(f'Test Phase {self.test_phase}: {test["description"]}')
            
            # Log results after a short delay
            self.create_timer(1.0, lambda: self.log_results(test['description']), timer_period_ns=0)
            
            self.test_phase += 1
        else:
            self.get_logger().info('Test sequence complete')
            self.test_timer.cancel()
    
    def apply_test_force(self, force, torque):
        """Apply a test force/torque"""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ft_sensor_link'
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]
        
        self.wrench_pub.publish(msg)
    
    def log_results(self, test_description):
        """Log the current state for validation"""
        self.get_logger().info(f'=== Results for: {test_description} ===')
        
        if self.last_cart_vel:
            self.get_logger().info(
                f'Cartesian velocity: '
                f'linear=[{self.last_cart_vel.linear.x:.3f}, '
                f'{self.last_cart_vel.linear.y:.3f}, '
                f'{self.last_cart_vel.linear.z:.3f}], '
                f'angular=[{self.last_cart_vel.angular.x:.3f}, '
                f'{self.last_cart_vel.angular.y:.3f}, '
                f'{self.last_cart_vel.angular.z:.3f}]'
            )
        else:
            self.get_logger().warn('No Cartesian velocity received')
        
        if self.last_trajectory and self.last_trajectory.points:
            point = self.last_trajectory.points[0]
            if point.velocities:
                vel_norm = np.linalg.norm(point.velocities)
                self.get_logger().info(
                    f'Joint velocities norm: {vel_norm:.3f}, '
                    f'velocities={[f"{v:.3f}" for v in point.velocities[:3]]}...'
                )
        else:
            self.get_logger().warn('No trajectory received')
        
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = NodeValidation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()