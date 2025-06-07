#!/usr/bin/env python3
"""Initialize robot to equilibrium position using trajectory controller,
then switch to velocity controller for admittance control."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
from rcl_interfaces.srv import GetParameters
from roboticstoolbox import DHRobot, RevoluteDH

class EquilibriumInitializer(Node):
    def __init__(self):
        super().__init__('equilibrium_initializer')
        
        self.declare_parameter('equilibrium.position', [0.1, 0.4, 0.5])
        self.declare_parameter('equilibrium.orientation', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('movement_duration', 12.0)
        
        self.movement_duration = self.get_parameter('movement_duration').value
        self.equilibrium_position = None
        self.current_positions = None
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        if not (self.setup_kinematics() and self.compute_equilibrium_joints()):
            raise RuntimeError('Initialization failed')
            
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        
        self.trajectory_client.wait_for_server()
        self.switch_controller_client.wait_for_service()
        
        while self.current_positions is None:
            rclpy.spin_once(self)
        
        self.execute_initialization()
        
    def setup_kinematics(self):
        param_client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        if not param_client.wait_for_service(timeout_sec=5.0):
            return False
            
        request = GetParameters.Request(names=['robot_description'])
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if not future.done() or not future.result() or not future.result().values[0].string_value:
            return False
            
        try:
            import tempfile, os
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(future.result().values[0].string_value)
                temp_path = f.name
            self.robot = rtb.ERobot.URDF(temp_path)
            os.unlink(temp_path)
            return True
        except:
            return False
        
    def compute_equilibrium_joints(self):
        eq_pos = self.get_parameter('equilibrium.position').value
        eq_ori = self.get_parameter('equilibrium.orientation').value
        
        target_pose_tool = SE3.Rt(UnitQuaternion([eq_ori[3], eq_ori[0], eq_ori[1], eq_ori[2]]).R, eq_pos)
        
        # Get transform from wrist_3_link to tool_payload
        q_zero = np.zeros(7)
        T_base_to_wrist3 = self.robot.fkine(q_zero, end=self.robot.links[5])
        T_base_to_tool = self.robot.fkine(q_zero, end=self.robot.links[6])
        T_wrist3_to_tool = T_base_to_wrist3.inv() @ T_base_to_tool
        
        # Convert target pose from tool_payload frame to wrist_3_link frame
        target_pose_wrist3 = target_pose_tool @ T_wrist3_to_tool.inv()
        
        # Create 6-DOF robot model for IK solving
        try:
            # UR5e DH parameters
            robot_6dof = DHRobot([
                RevoluteDH(d=0.1625, a=0, alpha=np.pi/2),      # shoulder_pan
                RevoluteDH(d=0, a=-0.425, alpha=0),            # shoulder_lift
                RevoluteDH(d=0, a=-0.3922, alpha=0),           # elbow
                RevoluteDH(d=0.1333, a=0, alpha=np.pi/2),      # wrist_1
                RevoluteDH(d=0.0997, a=0, alpha=-np.pi/2),     # wrist_2
                RevoluteDH(d=0.0996, a=0, alpha=0)             # wrist_3
            ], name="UR5e_6DOF")
            
            solution = robot_6dof.ikine_LM(target_pose_wrist3, q0=np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0]), ilimit=500, slimit=100)
            if solution.success:
                self.equilibrium_position = [float(q) for q in solution.q]
                self.get_logger().info(f'IK solved: {[f"{q:.3f}" for q in self.equilibrium_position]}')
                return True
        except:
            pass
            
        return False
        
    def joint_state_callback(self, msg):
        positions = [msg.position[msg.name.index(n)] for n in self.joint_names if n in msg.name]
        if len(positions) == 6:
            self.current_positions = positions
            
    def move_to_equilibrium(self):
        trajectory = JointTrajectory(joint_names=self.joint_names, points=[
            JointTrajectoryPoint(positions=self.current_positions, velocities=[0.0]*6, time_from_start=Duration(sec=0)),
            JointTrajectoryPoint(positions=self.equilibrium_position, velocities=[0.0]*6, 
                               time_from_start=Duration(sec=int(self.movement_duration), nanosec=int((self.movement_duration % 1) * 1e9)))
        ])
        
        send_goal_future = self.trajectory_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return result_future.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            
    def switch_to_velocity_controller(self):
        future = self.switch_controller_client.call_async(
            SwitchController.Request(deactivate_controllers=['scaled_joint_trajectory_controller'],
                                   activate_controllers=['forward_velocity_controller'],
                                   strictness=SwitchController.Request.BEST_EFFORT))
        rclpy.spin_until_future_complete(self, future)
        return future.result().ok
            
    def execute_initialization(self):
        if self.move_to_equilibrium():
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.switch_to_velocity_controller():
                self.get_logger().info('Robot initialized successfully')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = EquilibriumInitializer()
        rclpy.spin_once(node, timeout_sec=1.0)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()