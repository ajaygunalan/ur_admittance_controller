from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_admittance_controller"),
                "config",
                "admittance_config.yaml"
            ]),
            description="Path to the admittance configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "simulation_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_admittance_controller"),
                "config",
                "ur_admittance_controllers.yaml"
            ]),
            description="Path to the simulation configuration overrides",
        )
    )
    
    # Get arguments
    config_file = LaunchConfiguration("config_file")
    simulation_config = LaunchConfiguration("simulation_config")
    
    # Admittance node
    admittance_node = Node(
        package="ur_admittance_controller",
        executable="admittance_node",
        name="admittance_node",
        output="screen",
        parameters=[config_file, simulation_config],
        remappings=[
            # Ensure proper topic connections
            ("/wrist_ft_sensor", "/wrist_ft_sensor"),
            ("/joint_states", "/joint_states"),
            ("/scaled_joint_trajectory_controller/joint_trajectory", 
             "/scaled_joint_trajectory_controller/joint_trajectory"),
        ],
    )
    
    # Optional: Add a simple test publisher for F/T data
    test_wrench_publisher = Node(
        package="ros2",
        executable="run",
        arguments=[
            "topic", "pub", "--rate", "100",
            "/wrist_ft_sensor", "geometry_msgs/msg/WrenchStamped",
            "{header: {frame_id: 'ft_sensor_link'}, wrench: {force: {x: 0.0, y: 0.0, z: 10.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"
        ],
        condition=None,  # Set to IfCondition if you want to enable test mode
    )
    
    nodes_to_start = [admittance_node]
    
    return LaunchDescription(declared_arguments + nodes_to_start)