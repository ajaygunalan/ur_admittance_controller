from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to load the admittance controller into an existing ros2_control environment.
    This launch file should be used AFTER starting the simulation or robot.
    """
    # Spawn the admittance controller
    admittance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_admittance_controller', '--controller-manager', 'controller_manager'],
        output='screen',
    )
    
    return LaunchDescription([
        admittance_controller_spawner,
    ])
