#!/usr/bin/env python3
"""
UR Admittance Controller - Master Launch File

This launch file provides a comprehensive interface for starting the UR admittance
controller with various configurations. It handles controller spawning, parameter
configuration, and system monitoring.

BASIC USAGE:
    # Default configuration (simulation mode)
    ros2 launch ur_admittance_controller ur_admittance.launch.py

    # Real robot mode
    ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim:=false

    # Force/torque sensor only mode
    ros2 launch ur_admittance_controller ur_admittance.launch.py mode:=ft_only

    # Demo mode with preset configurations
    ros2 launch ur_admittance_controller ur_admittance.launch.py mode:=demo

ADVANCED OPTIONS:
    ur_type:=ur10e              # Robot model (ur3, ur3e, ur5, ur5e, ur10, ur10e)
    start_active:=false         # Start controller inactive
    demo_preset:=soft_impedance # Demo configuration preset
    controller_name:=my_controller # Custom controller name
    ft_sensor_name:=custom_ft   # Custom F/T sensor name

Author: UR Robotics Team
Date: 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate the launch description for the UR admittance controller.
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    # Declare launch arguments with descriptions
    declared_arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="full",
            choices=["full", "ft_only", "controller_only", "demo"],
            description="Launch mode: full=everything, ft_only=just F/T sensor, controller_only=just admittance, demo=with presets"
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Use simulation (true) or real robot (false)"
        ),
        DeclareLaunchArgument(
            "ur_type", 
            default_value="ur5e",
            description="UR robot type (ur3, ur3e, ur5, ur5e, ur10, ur10e)"
        ),
        DeclareLaunchArgument(
            "start_active",
            default_value="true", 
            description="Start controller in active state"
        ),
        DeclareLaunchArgument(
            "controller_name",
            default_value="ur_admittance_controller",
            description="Name of the admittance controller"
        ),
        DeclareLaunchArgument(
            "demo_preset",
            default_value="soft_impedance",
            choices=["pure_admittance", "soft_impedance", "stiff_z", "conservative"],
            description="Demo preset configuration"
        ),
        DeclareLaunchArgument(
            "ft_sensor_name",
            default_value="wrist_ft_sensor",
            description="F/T sensor hardware interface name"
        ),
    ]

    mode = LaunchConfiguration("mode")
    use_sim = LaunchConfiguration("use_sim")
    ur_type = LaunchConfiguration("ur_type")
    start_active = LaunchConfiguration("start_active") 
    controller_name = LaunchConfiguration("controller_name")
    demo_preset = LaunchConfiguration("demo_preset")
    ft_sensor_name = LaunchConfiguration("ft_sensor_name")

    config_file = PathJoinSubstitution([
        FindPackageShare("ur_admittance_controller"),
        "config", 
        "admittance_config.yaml"
    ])
    
    # Separate config file for controller manager - different for sim vs hardware
    controller_config_file_sim = PathJoinSubstitution([
        FindPackageShare("ur_admittance_controller"),
        "config", 
        "ur_admittance_controllers.yaml"
    ])
    
    controller_config_file_hw = PathJoinSubstitution([
        FindPackageShare("ur_admittance_controller"),
        "config", 
        "ur_admittance_controllers_hw.yaml"
    ])
    
    # Select config based on use_sim
    controller_config_file = PythonExpression([
        "'", controller_config_file_sim, "' if '", use_sim, "' == 'true' else '", controller_config_file_hw, "'"
    ])

    # Force/Torque sensor broadcaster is not needed in Gazebo simulation
    # Gazebo publishes F/T sensor data directly to /wrist_ft_sensor topic
    # This node is kept as a placeholder for real robot mode
    ft_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controller_config_file,
            "--controller-manager-timeout", "300"
        ],
        output="screen",
        # Disable for simulation mode since Gazebo publishes directly to /wrist_ft_sensor
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'controller_only' or '", use_sim, "' == 'true'"]))
    )

    # Load admittance controller (initially inactive)
    # Delayed by 2 seconds to ensure F/T sensor is ready
    load_admittance_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller_name,
                    "--controller-manager", "/controller_manager", 
                    "--param-file", controller_config_file,
                    "--inactive",  # Start inactive for safe initialization
                    "--controller-manager-timeout", "300"
                ],
                output="screen",
                parameters=[{
                    "use_sim": use_sim,
                    "ur_type": ur_type
                }]
            )
        ],
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'ft_only'"]))
    )

    activate_admittance_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner", 
                arguments=[
                    controller_name,
                    "--controller-manager", "/controller_manager",
                    "--activate",
                    "--controller-manager-timeout", "300"
                ],
                output="screen"
            )
        ],
        condition=IfCondition(PythonExpression(["'", start_active, "' == 'true' and '", mode, "' != 'ft_only'"]))
    )

    # Demo configuration presets
    # Each preset defines parameter values for different use cases
    demo_configs = {
        # Pure admittance mode - zero stiffness, compliant behavior
        "pure_admittance": [
            "admittance.stiffness", "[0.0,0.0,0.0,0.0,0.0,0.0]",
            "admittance.mass", "[8.0,8.0,8.0,0.8,0.8,0.8]", 
            "admittance.damping_ratio", "[0.8,0.8,0.8,0.8,0.8,0.8]"
        ],
        # Soft impedance mode - low stiffness for gentle interaction
        "soft_impedance": [
            "admittance.stiffness", "[50.0,50.0,50.0,5.0,5.0,5.0]",
            "admittance.mass", "[8.0,8.0,8.0,0.8,0.8,0.8]",
            "admittance.damping_ratio", "[0.8,0.8,0.8,0.8,0.8,0.8]"
        ],
        # Stiff Z-axis mode - compliant in XY, stiff in Z
        "stiff_z": [
            "admittance.stiffness", "[0.0,0.0,200.0,0.0,0.0,0.0]", 
            "admittance.mass", "[8.0,8.0,8.0,0.8,0.8,0.8]",
            "admittance.damping_ratio", "[0.8,0.8,0.8,0.8,0.8,0.8]"
        ],
        # Conservative mode - higher mass/damping, lower velocities
        "conservative": [
            "admittance.mass", "[12.0,12.0,12.0,1.2,1.2,1.2]",
            "admittance.damping_ratio", "[1.2,1.2,1.2,1.2,1.2,1.2]",
            "max_linear_velocity", "0.3",
            "max_angular_velocity", "0.8"
        ]
    }

    demo_preset_actions = []
    for preset_name, params in demo_configs.items():
        for i in range(0, len(params), 2):
            demo_preset_actions.append(
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', f'/{controller_name}', params[i], params[i+1]],
                    output='screen',
                    condition=IfCondition(PythonExpression(["'", demo_preset, "' == '", preset_name, "'"]))
                )
            )

    apply_demo_preset = TimerAction(
        period=6.0,
        actions=demo_preset_actions,
        condition=IfCondition(PythonExpression(["'", mode, "' == 'demo'"]))
    )

    # System status monitor
    # Periodically checks controller health and reports status
    system_status = TimerAction(
        period=8.0,  # Start after 8 seconds
        actions=[
            Node(
                package="ur_admittance_controller",
                executable="system_status.py",
                name="ur_admittance_status",
                output="screen",
                parameters=[{
                    "focus_controller": controller_name,
                    "check_period": 15.0,  # Check every 15 seconds
                    "use_sim": use_sim
                }]
            )
        ]
    )

    instructions = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c", 
                    "echo '🎯 UR Admittance Controller Launched!' && "
                    "echo 'Mode: Full System' && "
                    "echo '' && "
                    "echo '📊 Quick Checks:' && "
                    "echo '  F/T Sensor: ros2 topic echo /ft_sensor_readings --once' && "
                    "echo '  Controllers: ros2 control list_controllers' && "
                    "echo '  Parameters: ros2 param list /ur_admittance_controller' && "
                    "echo '' && "
                    "echo '🧪 Testing (Gazebo):' && "
                    "echo '  1. Press F key to enable Force mode' && "
                    "echo '  2. Click and drag robot end-effector' && "
                    "echo '  3. Robot moves compliantly!' && "
                    "echo ''"
                ],
                output="screen",
                condition=IfCondition(PythonExpression(["'", mode, "' == 'full'"]))
            ),
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    f"echo '🎯 Demo Mode Active!' && "
                    f"echo 'Preset: {demo_preset}' && "
                    f"echo '' && "
                    f"echo 'Try pushing the robot - it will react based on the preset!' && "
                    f"echo ''"
                ],
                output="screen", 
                condition=IfCondition(PythonExpression(["'", mode, "' == 'demo'"]))
            )
        ]
    )

    # Build and return the complete launch description
    return LaunchDescription(declared_arguments + [
        ft_sensor_broadcaster,
        load_admittance_controller, 
        activate_admittance_controller,
        apply_demo_preset,
        system_status,
        instructions
    ])
