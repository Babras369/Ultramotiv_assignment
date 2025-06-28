from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get URDF and config paths
    model_pkg = 'cobot_trial2_description'
    controller_pkg = 'cobot2_new'

    xacro_file = os.path.join(
        get_package_share_directory(model_pkg),
        'urdf',
        'cobot_trial2.xacro'
    )

    controller_file = os.path.join(
        get_package_share_directory(controller_pkg),
        'config',
        'ros2_controllers.yaml'
    )

    # Robot description (from xacro)
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file
    ])
    robot_description = {'robot_description': robot_description_content}

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # ros2_control_node is critical!
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_file],
        output="screen"
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'cobot_trial2'],
        output='screen'
    )

    # Controller spawners
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # IMPORTANT: Match this with your actual controller name in ros2_controllers.yaml
    joint_state_controller_arma_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_controller_arma_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        ros2_control_node,             # ✅ Needed for ros2_control to function
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster,
        joint_state_controller_arma_controller                # ✅ Use your actual controller name
    ])
