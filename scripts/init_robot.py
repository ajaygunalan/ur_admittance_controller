#!/usr/bin/env python3
"""
Initialize robot to equilibrium position using trajectory controller,
then switch to velocity controller for admittance control.

Reads equilibrium pose from Cartesian space parameters (same as admittance_node)
and converts to joint space using KDL inverse kinematics.
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
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
from rcl_interfaces.srv import GetParameters


class EquilibriumInitializer(Node):
    def __init__(self):
        super().__init__('equilibrium_initializer')
        
        # Declare parameters matching admittance_node
        self.declare_parameter('equilibrium.position', [0.1, 0.4, 0.5])  # Cartesian XYZ
        self.declare_parameter('equilibrium.orientation', [0.0, 0.0, 0.0, 1.0])  # Quaternion xyzw
        self.declare_parameter('movement_duration', 12.0)
        self.declare_parameter('robot_description', '')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'tool_payload')  # Match admittance controller
        
        # Get parameters
        self.movement_duration = self.get_parameter('movement_duration').value
        
        # Robot model
        self.robot = None
        
        # Equilibrium joint position (will be computed from Cartesian)
        self.equilibrium_position = None
        
        # Joint names for UR robots
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Setup KDL kinematics
        if not self.setup_kinematics():
            self.get_logger().error('Failed to setup kinematics')
            raise RuntimeError('Kinematics initialization failed')
            
        # Convert Cartesian equilibrium to joint space
        if not self.compute_equilibrium_joints():
            self.get_logger().error('Failed to compute equilibrium joint configuration')
            raise RuntimeError('Equilibrium computation failed')
        
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
        
    def setup_kinematics(self):
        """Setup robot kinematics using roboticstoolbox."""
        self.get_logger().info('Getting robot_description from /robot_state_publisher...')
        
        # Create parameter client to get URDF
        param_client = self.create_client(
            GetParameters,
            '/robot_state_publisher/get_parameters'
        )
        
        if not param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('robot_state_publisher not available')
            return False
            
        # Get robot_description parameter
        request = GetParameters.Request()
        request.names = ['robot_description']
        
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if not future.done() or future.result() is None:
            self.get_logger().error('Failed to get robot_description')
            return False
            
        response = future.result()
        if len(response.values) == 0 or not response.values[0].string_value:
            self.get_logger().error('robot_description is empty')
            return False
            
        urdf_string = response.values[0].string_value
        
        # roboticstoolbox requires a file path
        import tempfile
        import os
        
        try:
            # Write URDF to temporary file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(urdf_string)
                temp_urdf_path = f.name
            
            # Load robot from URDF
            self.robot = rtb.ERobot.URDF(temp_urdf_path)
            
            self.get_logger().info(f'Loaded robot from URDF: {self.robot.name}')
            self.get_logger().info(f'Robot has {self.robot.n} DOF')
            
            # Clean up temp file
            os.unlink(temp_urdf_path)
            
            # Note: roboticstoolbox only recognizes 6 actuated joints
            # The F/T sensor joint (with zero limits) is not included
            if self.robot.n != 6:
                self.get_logger().warn(f'Expected 6 DOF but got {self.robot.n}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF: {str(e)}')
            if 'temp_urdf_path' in locals():
                try:
                    os.unlink(temp_urdf_path)
                except:
                    pass
            return False
        
    def compute_equilibrium_joints(self):
        """Convert Cartesian equilibrium pose to joint configuration using IK."""
        # Get Cartesian equilibrium parameters
        eq_pos = self.get_parameter('equilibrium.position').value
        eq_ori = self.get_parameter('equilibrium.orientation').value
        
        self.get_logger().info(f'Target equilibrium position: {eq_pos}')
        self.get_logger().info(f'Target equilibrium orientation (quat xyzw): {eq_ori}')
        
        # Create target pose
        target_position = np.array(eq_pos)
        # Convert quaternion from [x,y,z,w] to [w,x,y,z] for UnitQuaternion
        target_orientation = UnitQuaternion([eq_ori[3], eq_ori[0], eq_ori[1], eq_ori[2]])
        target_pose = SE3.Rt(target_orientation.R, target_position)
        
        # Initial guess for IK (6 joints for UR robot)
        q_init = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        
        # Solve IK
        try:
            # Use the configured tip_link as the end effector for IK
            tip_link = self.get_parameter('tip_link').value
            self.get_logger().info(f'Solving IK for tip_link: {tip_link}')
            solution = self.robot.ikine_LM(target_pose, q0=q_init, end=tip_link, ilimit=500, slimit=100)
            
            if solution.success:
                self.equilibrium_position = np.array(solution.q).flatten().tolist()
                
                # Verify with forward kinematics
                fk_pose = self.robot.fkine(solution.q)
                pos_error = np.linalg.norm(fk_pose.t - target_position)
                
                self.get_logger().info(f'IK converged in {solution.iterations} iterations')
                self.get_logger().info(f'Position error: {pos_error:.6f} m')
                self.get_logger().info(f'Joint angles (rad): {[f"{q:.3f}" for q in self.equilibrium_position]}')
                
                # IK is now computed for the configured tip_link (tool_payload)
                # This ensures consistency with the admittance controller's kinematic chain
                
                return True
            else:
                self.get_logger().error(f'IK failed: {solution.reason}')
                # Try alternative initial guess
                q_init2 = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
                solution = self.robot.ikine_LM(target_pose, q0=q_init2, end=tip_link, ilimit=500, slimit=100)
                
                if solution.success:
                    self.equilibrium_position = np.array(solution.q).flatten().tolist()
                    self.get_logger().info('IK succeeded with alternative initial guess')
                    return True
                    
        except Exception as e:
            self.get_logger().error(f'IK exception: {str(e)}')
            
        # Fall back to default configuration
        self.get_logger().warn('Using fallback joint configuration')
        self.equilibrium_position = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        return True
        
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