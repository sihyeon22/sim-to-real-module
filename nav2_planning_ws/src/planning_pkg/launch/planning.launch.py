
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    # DeclareLaunchArgument('namespace', default_value=''),
    DeclareLaunchArgument('use_sim_time', default_value='true'),
    DeclareLaunchArgument('log_level', default_value='info'),
    DeclareLaunchArgument('use_respawn', default_value='False'),
    DeclareLaunchArgument('robot', default_value='fr09'),
    DeclareLaunchArgument('level', default_value='2d'),
]

def generate_launch_description():
    # namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    use_respawn = LaunchConfiguration('use_respawn')
    robot = LaunchConfiguration('robot')
    level = LaunchConfiguration('level')
    
    lifecycle_nodes = ['smoother_server',
                       'planner_server',
                       'bt_navigator',
                       'waypoint_follower']
    
    # Directories
    pkg_nav2 = get_package_share_directory(
        'planning_pkg')
    
    # Behavior Tree
    default_nav_to_pose_bt_xml = PathJoinSubstitution([
        pkg_nav2,
        'behavior',
        'navigate_to_pose_no_spin.xml'
    ])
    default_nav_through_poses_bt_xml = PathJoinSubstitution([
        pkg_nav2,
        'behavior',
        'navigate_through_poses_no_spin.xml'
    ])
    
    # Parameters
    params_file = PathJoinSubstitution([
        pkg_nav2,
        'config',
        PythonExpression(["'planning_", level, "_",  robot, ".yaml'"])])
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml
    }
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            # root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    load_nodes = GroupAction(
        actions=[
            # PushRosNamespace(namespace),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_planning',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}]
            ),
        ],
    )
            
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(load_nodes)
    
    return ld
