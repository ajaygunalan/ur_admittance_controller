from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Set impedance mode parameters
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/ur_admittance_controller', 
                 'admittance.stiffness', '[100.0,100.0,100.0,10.0,10.0,10.0]'],
            output='screen'
        ),
        
        # Show configuration
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['echo', 'Impedance mode activated! Push the robot and watch it spring back.'],
                    output='screen'
                )
            ]
        )
    ])
