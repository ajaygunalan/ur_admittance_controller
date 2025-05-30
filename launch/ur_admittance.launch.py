#!/usr/bin/env python3
"""UR Admittance Node Launch"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur_admittance_controller",
            executable="admittance_node",
            parameters=[PathJoinSubstitution([
                FindPackageShare("ur_admittance_controller"),
                "config", "admittance_config.yaml"
            ])],
            output="screen"
        )
    ])