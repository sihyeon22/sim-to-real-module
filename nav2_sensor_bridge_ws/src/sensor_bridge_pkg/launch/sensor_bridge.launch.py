from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='true',
            description='true: simulation (CARLA), false: real vehicle',
        ),
        Node(
            package='sensor_bridge_pkg',
            executable='sensor_bridge',
            name='sensor_bridge',
            output='screen',
            parameters=[{
                'sim_mode': LaunchConfiguration('sim_mode'),
            }],
        ),
    ])
