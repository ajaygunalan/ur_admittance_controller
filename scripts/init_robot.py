#!/usr/bin/env python3
"""
Initialize robot to equilibrium position using trajectory controller,
then switch to velocity controller for admittance control.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController
import time


class EquilibriumInitializer(Node):
    def __init__(self):
        super().__init__('equilibrium_initializer')
        
        # Parameters
        self.declare_parameter('equilibrium_position', [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        self.declare_parameter('movement_duration', 6.0)
        
        self.equilibrium_position = self.get_parameter('equilibrium_position').value
        self.movement_duration = self.get_parameter('movement_duration').value
        
        # Joint names for UR robots
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Subscribe to joint states
        self.current_positions = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action client for trajectory controller
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Service client for controller switching
        self.switch_controller_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services...')
        self.trajectory_client.wait_for_server()
        self.switch_controller_client.wait_for_service()
        
        # Wait for joint states
        self.get_logger().info('Waiting for joint states...')
        while self.current_positions is None and rclpy.ok():
            rclpy.spin_once(self)
            
        # Execute initialization sequence
        self.execute_initialization()
        
    def joint_state_callback(self, msg):
        """Store current joint positions."""
        positions = []
        for joint_name in self.joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
        
        if len(positions) == len(self.joint_names):
            self.current_positions = positions
            
    def move_to_equilibrium(self):
        """Move robot to equilibrium position using trajectory controller."""
        self.get_logger().info('Moving to equilibrium position...')
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Start point (current position)
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_positions
        start_point.velocities = [0.0] * len(self.joint_names)
        start_point.time_from_start = Duration(sec=0)
        
        # End point (equilibrium position)
        end_point = JointTrajectoryPoint()
        end_point.positions = self.equilibrium_position
        end_point.velocities = [0.0] * len(self.joint_names)
        end_point.time_from_start = Duration(sec=int(self.movement_duration))
        
        trajectory.points = [start_point, end_point]
        
        # Send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info('Sending trajectory goal...')
        send_goal_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False
            
        # Wait for result
        self.get_logger().info('Executing trajectory...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Successfully moved to equilibrium position')
            return True
        else:
            self.get_logger().error(f'Trajectory execution failed with error code: {result.result.error_code}')
            return False
            
    def switch_to_velocity_controller(self):
        """Switch from trajectory controller to velocity controller."""
        self.get_logger().info('Switching to velocity controller...')
        
        request = SwitchController.Request()
        request.deactivate_controllers = ['scaled_joint_trajectory_controller']
        request.activate_controllers = ['forward_velocity_controller']
        request.strictness = SwitchController.Request.BEST_EFFORT
        
        future = self.switch_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.ok:
            self.get_logger().info('Successfully switched to forward_velocity_controller')
            return True
        else:
            self.get_logger().error('Failed to switch controllers')
            return False
            
    def execute_initialization(self):
        """Execute the full initialization sequence."""
        # Step 1: Move to equilibrium
        if not self.move_to_equilibrium():
            self.get_logger().error('Failed to move to equilibrium position')
            return
            
        # Small delay to ensure motion is complete
        time.sleep(0.5)
        
        # Step 2: Switch controllers
        if not self.switch_to_velocity_controller():
            self.get_logger().error('Failed to switch controllers')
            return
            
        self.get_logger().info('Initialization complete! Ready for admittance control.')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        initializer = EquilibriumInitializer()
        # Keep node alive briefly to ensure messages are sent
        time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()