from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_admittance_controller",
            description="Package with the controller's configuration in 'config' folder",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/xacro files",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup. If changed, then also joint names in the controllers' configuration have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="Robot IP address",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Choose whether to use simulation (Gazebo) or real robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace of the robot",
        )
    )

    # Initialize arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_ip = LaunchConfiguration("robot_ip")
    use_sim = LaunchConfiguration("use_sim")
    namespace = LaunchConfiguration("namespace")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=",
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_sim:=",
            use_sim,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Load controllers
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # Controller manager with the admittance controller
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        namespace=namespace,
        output="screen",
    )

    # Start controllers
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        namespace=namespace,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
        namespace=namespace,
    )

    ur_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "controller_manager"],
        namespace=namespace,
    )
    
    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_admittance_controller", "--controller-manager", "controller_manager"],
        namespace=namespace,
    )

    # Force-torque sensor data publisher
    force_torque_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["force_torque_sensor_broadcaster", "-c", "controller_manager"],
        namespace=namespace,
    )

    # Delay start of controllers after joint state broadcaster
    delay_joint_state_broadcaster_after_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_ur_controller_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ur_controller_spawner],
        )
    )
    
    delay_force_torque_sensor_broadcaster_after_ur_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ur_controller_spawner,
            on_exit=[force_torque_sensor_broadcaster_spawner],
        )
    )
    
    delay_admittance_controller_after_force_torque_sensor = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=force_torque_sensor_broadcaster_spawner,
            on_exit=[admittance_controller_spawner],
        )
    )

    nodes = [
        controller_manager_node,
        robot_state_pub_node,
        delay_joint_state_broadcaster_after_controller_manager,
        delay_ur_controller_spawner_after_joint_state_broadcaster,
        delay_force_torque_sensor_broadcaster_after_ur_controller,
        delay_admittance_controller_after_force_torque_sensor,
    ]

    return LaunchDescription(declared_arguments + nodes)
