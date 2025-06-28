import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",  # Changed to your config file
            description="RViz configuration file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def load_yaml(file_path):
    """Load YAML file and return its contents."""
    if os.path.exists(file_path):
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    else:
        print(f"Warning: {file_path} not found.")
        return {}


def launch_setup(context, *args, **kwargs):
    
    # Get package directory
    pkg_share = get_package_share_directory('cobot_trial2_description')
    
    # File paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'cobot_trial2.xacro')
    srdf_file = os.path.join(pkg_share, 'urdf', 'cobot_trial2.srdf')
    kinematics_file = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    ompl_file = os.path.join(pkg_share, 'config', 'ompl_planning.yaml')
    controllers_file = os.path.join(pkg_share, 'config', 'moveit_controllers.yaml')
    joint_limits_file = os.path.join(pkg_share, 'config', 'joint_limits.yaml')
    pilz_file = os.path.join(pkg_share, 'config', 'pilz_cartesian_limits.yaml')
    
    # Process robot description
    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}
    
    # Load SRDF
    robot_description_semantic = {}
    if os.path.exists(srdf_file):
        with open(srdf_file, 'r') as f:
            robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Load other configs
    robot_description_kinematics = load_yaml(kinematics_file)
    ompl_planning_pipeline_config = load_yaml(ompl_file)
    moveit_controllers = load_yaml(controllers_file)
    joint_limits = load_yaml(joint_limits_file)
    pilz_cartesian_limits = load_yaml(pilz_file)
    
    # Configure multiple planning pipelines
    planning_pipelines_config = {
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_pipeline_config.get('ompl', {}),
        'pilz_industrial_motion_planner': {
            'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
            'request_adapters': '',
            'default_planner_config': 'PTP'
        }
    }
    
    # Planning scene monitor parameters
    planning_scene_monitor = {
        'planning_scene_monitor': {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        }
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines_config,
            moveit_controllers,
            joint_limits,
            pilz_cartesian_limits,
            planning_scene_monitor,
            {"use_sim_time": True}
        ],
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("cobot_trial2_description"), "config", rviz_base]  # Your package
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines_config,
            moveit_controllers,
            joint_limits,
            pilz_cartesian_limits,
            planning_scene_monitor,
            {"use_sim_time": True}
        ],
    )

    # Static TF - Change this to match your robot's base frame
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],  # Change to your base frame
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
    )

    # ros2_control using FakeSystem as hardware (or remove if using Gazebo)
    ros2_controllers_path = os.path.join(
        get_package_share_directory("cobot_trial2_description"),  # Your package
        "config",
        "ros2_controllers.yaml",  # Your controllers config
    )
    
    # Comment out or remove this if you're using Gazebo (since Gazebo provides the hardware interface)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Change this to match your arm controller name
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],  # Your controller name
    )

    nodes_to_start = [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,  # Comment out if using Gazebo
        #joint_state_broadcaster_spawner,
        #arm_controller_spawner,
    ]

    return nodes_to_start