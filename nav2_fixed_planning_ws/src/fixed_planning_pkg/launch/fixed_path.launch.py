from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'),
    DeclareLaunchArgument('log_level', default_value='info'),
    DeclareLaunchArgument('use_respawn', default_value='False'),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    use_respawn = LaunchConfiguration('use_respawn')

    lifecycle_nodes = ['planner_server']

    pkg = get_package_share_directory('fixed_planning_pkg')
    params_file = PathJoinSubstitution([pkg, 'config', 'fixed_planning_2d_fr09.yaml'])
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True)

    return LaunchDescription(ARGUMENTS + [
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
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_fixed_planning',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': lifecycle_nodes},
            ],
        ),
        Node(
            package='fixed_planning_pkg',
            executable='fixed_path_follower',
            name='fixed_path_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
