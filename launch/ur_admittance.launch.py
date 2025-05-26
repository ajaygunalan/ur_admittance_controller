#!/usr/bin/env python3
"""
UR Admittance Controller - MASTER Launch File
This is the ONLY launch file you need! Handles everything.

BASIC USAGE:
  # Start everything (simulation)
  ros2 launch ur_admittance_controller ur_admittance.launch.py

  # Real robot  
  ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim:=false

  # Just F/T sensor
  ros2 launch ur_admittance_controller ur_admittance.launch.py mode:=ft_only

  # Demo mode (with impedance presets)
  ros2 launch ur_admittance_controller ur_admittance.launch.py mode:=demo

ADVANCED OPTIONS:
  ur_type:=ur10e                    # Robot type
  start_active:=false               # Load but don't activate
  demo_preset:=soft_impedance       # Demo configuration
  controller_name:=my_controller    # Custom controller name
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
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

    # Configuration
    mode = LaunchConfiguration("mode")
    use_sim = LaunchConfiguration("use_sim")
    ur_type = LaunchConfiguration("ur_type")
    start_active = LaunchConfiguration("start_active") 
    controller_name = LaunchConfiguration("controller_name")
    demo_preset = LaunchConfiguration("demo_preset")
    ft_sensor_name = LaunchConfiguration("ft_sensor_name")

    # Config file path - FIXED to use our single config
    config_file = PathJoinSubstitution([
        FindPackageShare("ur_admittance_controller"),
        "config", 
        "admittance_config.yaml"
    ])

    # ==========================================================================
    # F/T SENSOR BROADCASTER (for modes: full, ft_only, demo)
    # ==========================================================================
    ft_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", config_file,
            "--controller-manager-timeout", "300"
        ],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("mode").__eq__(TextSubstitution(text="controller_only")))
    )

    # ==========================================================================
    # ADMITTANCE CONTROLLER (for modes: full, controller_only, demo) 
    # ==========================================================================
    
    # Load controller (inactive first)
    load_admittance_controller = TimerAction(
        period=2.0,  # Wait for F/T sensor if needed
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller_name,
                    "--controller-manager", "/controller_manager", 
                    "--param-file", config_file,
                    "--inactive",
                    "--controller-manager-timeout", "300"
                ],
                output="screen",
                parameters=[{
                    "use_sim": use_sim,
                    "ur_type": ur_type
                }]
            )
        ],
        condition=UnlessCondition(TextSubstitution(text="ft_only").__eq__(mode))
    )

    # Activate controller (if requested)
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
        condition=IfCondition(start_active).__and__(UnlessCondition(TextSubstitution(text="ft_only").__eq__(mode)))
    )

    # ==========================================================================
    # DEMO MODE PRESETS (only for mode: demo)
    # ==========================================================================
    
    # Demo preset configurations
    demo_configs = {
        "pure_admittance": [
            "admittance.stiffness", "[0.0,0.0,0.0,0.0,0.0,0.0]",
            "admittance.mass", "[8.0,8.0,8.0,0.8,0.8,0.8]", 
            "admittance.damping_ratio", "[0.8,0.8,0.8,0.8,0.8,0.8]"
        ],
        "soft_impedance": [
            "admittance.stiffness", "[50.0,50.0,50.0,5.0,5.0,5.0]",
            "admittance.mass", "[8.0,8.0,8.0,0.8,0.8,0.8]",
            "admittance.damping_ratio", "[0.8,0.8,0.8,0.8,0.8,0.8]"
        ],
        "stiff_z": [
            "admittance.stiffness", "[0.0,0.0,200.0,0.0,0.0,0.0]", 
            "admittance.mass", "[8.0,8.0,8.0,0.8,0.8,0.8]",
            "admittance.damping_ratio", "[0.8,0.8,0.8,0.8,0.8,0.8]"
        ],
        "conservative": [
            "admittance.mass", "[12.0,12.0,12.0,1.2,1.2,1.2]",
            "admittance.damping_ratio", "[1.2,1.2,1.2,1.2,1.2,1.2]",
            "max_linear_velocity", "0.3",
            "max_angular_velocity", "0.8"
        ]
    }

    # Apply demo preset
    demo_preset_actions = []
    for preset_name, params in demo_configs.items():
        for i in range(0, len(params), 2):
            demo_preset_actions.append(
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', f'/{controller_name}', params[i], params[i+1]],
                    output='screen',
                    condition=IfCondition(TextSubstitution(text=preset_name).__eq__(demo_preset))
                )
            )

    apply_demo_preset = TimerAction(
        period=6.0,  # After controller is loaded and active
        actions=demo_preset_actions,
        condition=IfCondition(TextSubstitution(text="demo").__eq__(mode))
    )

    # ==========================================================================
    # SYSTEM STATUS & INSTRUCTIONS
    # ==========================================================================
    
    system_status = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ur_admittance_controller",
                executable="system_status.py",
                name="ur_admittance_status",
                output="screen",
                parameters=[{
                    "focus_controller": controller_name,
                    "check_period": 15.0,
                    "use_sim": use_sim
                }]
            )
        ]
    )

    # Usage instructions based on mode
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
                condition=IfCondition(TextSubstitution(text="full").__eq__(mode))
            ),
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "echo '🎯 Demo Mode Active!' && "
                    "echo f'Preset: {demo_preset}' && "
                    "echo '' && "
                    "echo 'Try pushing the robot - it will react based on the preset!' && "
                    "echo ''"
                ],
                output="screen", 
                condition=IfCondition(TextSubstitution(text="demo").__eq__(mode))
            )
        ]
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    
    return LaunchDescription(declared_arguments + [
        # Core components
        ft_sensor_broadcaster,
        load_admittance_controller, 
        activate_admittance_controller,
        
        # Demo mode
        apply_demo_preset,
        
        # Monitoring
        system_status,
        instructions
    ])
