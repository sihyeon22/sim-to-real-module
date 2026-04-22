
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
    
    lifecycle_nodes = ['controller_server',
                       'behavior_server',
                       'velocity_smoother']
    
    # Directories
    pkg_nav2 = get_package_share_directory(
        'control_pkg')
    
    params_file = PathJoinSubstitution([
        pkg_nav2,
        'config',
        PythonExpression(["'control_", level, "_",  robot, ".yaml'"])])
    param_substitutions = {
        'use_sim_time': use_sim_time,
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
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('/cmd_vel', 'cmd_vel_nav')]
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_control',
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
