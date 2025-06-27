import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    slowdown = LaunchConfiguration('slowdown')
    robot_controller = LaunchConfiguration('robot_controller')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    
    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'
    )
    
    declare_use_fake_hardware_cmd = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Start robot with fake hardware mirroring command to its states'
    )
    
    declare_fake_sensor_commands_cmd = DeclareLaunchArgument(
        'fake_sensor_commands',
        default_value='false',
        description='Enable fake command interfaces for sensors used for simple simulations'
    )
    
    declare_slowdown_cmd = DeclareLaunchArgument(
        'slowdown',
        default_value='3.0',
        description='Slowdown factor of the RRbot'
    )
    
    declare_robot_controller_cmd = DeclareLaunchArgument(
        'robot_controller',
        default_value='joint_trajectory_controller',
        description='Robot controller to start'
    )
    
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("cobot_trial2_description"),
            "urdf",
            "cobot_trial2.urdf.xacro"
        ]),
        " ",
        "use_ros2_control:=", use_ros2_control,
        " ",
        "use_fake_hardware:=", use_fake_hardware,
        " ",
        "fake_sensor_commands:=", fake_sensor_commands,
        " ",
        "slowdown:=", slowdown,
    ])
    
    robot_description = {"robot_description": robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # RViz2 node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("cobot_trial2_description"),
        "config",
        "view_robot.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Gazebo Classic launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "verbose": "true",
            "pause": "false",
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "cobot_trial2",
            "-x", "0.0",
            "-y", "0.0", 
            "-z", "0.0"
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Robot Controller Spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Delay RViz start until after other nodes
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[rviz_node],
        )
    )
    
    # Delay controller spawning until after joint state broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    # Optional: Add robot controller configuration
    robot_controllers_file = PathJoinSubstitution([
        FindPackageShare("cobot_trial2_description"),
        "config",
        "ros2_controllers.yaml"
    ])
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers_file,
            {"use_sim_time": use_sim_time}
        ],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    
    # Optional: Add a simple trajectory publisher for testing
    trajectory_publisher_node = Node(
        package="cobot_trial2_description",  # Replace with your package name
        executable="trajectory_publisher.py",  # Create this script for testing
        name="trajectory_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=LaunchConfiguration('start_trajectory_publisher', default='false')
    )
    
    # Nodes to launch
    nodes = [
        declare_use_sim_time_cmd,
        declare_use_ros2_control_cmd,
        declare_use_fake_hardware_cmd,
        declare_fake_sensor_commands_cmd,
        declare_slowdown_cmd,
        declare_robot_controller_cmd,
        
        # Core nodes
        robot_state_publisher_node,
        control_node,
        
        # Gazebo
        gazebo_launch,
        spawn_entity_node,
        
        # Controllers
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        
        # RViz (delayed)
        delay_rviz_after_joint_state_broadcaster_spawner,
        
        # Optional trajectory publisher
        # trajectory_publisher_node,
    ]
    
    return LaunchDescription(nodes)
