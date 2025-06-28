import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Paths
    pkg_share = FindPackageShare('your_robot_moveit_config')
    
    # Robot description
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([
            FindPackageShare('your_robot_description'),
            'urdf',
            'robot.urdf.xacro'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # MoveIt2 SRDF
    robot_description_semantic_content = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_share,
            'srdf',
            'robot.srdf.xacro'
        ])
    ])
    
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }
    
    # Kinematics config
    kinematics_yaml = load_yaml('your_robot_moveit_config', 'config/kinematics.yaml')
    
    # Planning config
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    
    # Trajectory execution
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # Controllers
    moveit_controllers = PathJoinSubstitution([
        pkg_share,
        'config',
        'moveit_controllers.yaml'
    ])
    
    # Planning scene monitor
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # RViz config
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'moveit.rviz'
    ])
    
    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        rviz_node,
    ])

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    # try:
    #     with open(absolute_file_path, 'r') as file:
    #         return yaml.safe_load(file)
    # except EnvironmentError:
    #     return None