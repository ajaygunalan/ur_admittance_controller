from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    # Get arguments
    config_file = LaunchConfiguration("config_file")
    
    # Admittance node
    admittance_node = Node(
        package="ur_admittance_controller",
        executable="admittance_node",
        name="admittance_node",
        output="screen",
        parameters=[config_file],
        remappings=[
            ("/wrist_ft_sensor", "/wrist_ft_sensor"),
            ("/joint_states", "/joint_states"),
            ("/scaled_joint_trajectory_controller/joint_trajectory", 
             "/scaled_joint_trajectory_controller/joint_trajectory"),
        ],
    )
    
    nodes_to_start = [admittance_node]
    
    return LaunchDescription(declared_arguments + nodes_to_start)