from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Packages
    description_pkg = 'cobot_trial2_description'
    controller_pkg = 'cobot2_new'

    # File paths
    xacro_file = os.path.join(
        get_package_share_directory(description_pkg),
        'urdf',
        'cobot_trial2.xacro'
    )

    srdf_file = os.path.join(
        get_package_share_directory(controller_pkg),
        'config',
        'cobot_trial2.srdf'
    )

    controllers_file = os.path.join(
        get_package_share_directory(controller_pkg),
        'config',
        'ros2_controllers.yaml'
    )

    kinematics_file = os.path.join(
        get_package_share_directory(controller_pkg),
        'config',
        'kinematics.yaml'
    )

    # Robot description (URDF from Xacro)
    robot_description_content = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # SRDF for MoveIt
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command(['cat', srdf_file]),
            value_type=str
        )
    }

    # Kinematics config
    kinematics = {'robot_description_kinematics': kinematics_file}

    # Nodes

    # Robot state publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # ros2_control node (loads controllers and interfaces)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen'
    )

    # Joint state broadcaster
    js_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Actual arm controller — replace with your controller name (check YAML file)
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_controller_arma_controller'],  # <- change if needed
        output='screen'
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory(controller_pkg),
        'config',
        'moveit.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        control_node,
        js_broadcaster,
        arm_controller,
        rviz_node
    ])
