from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='tb4_1',
            description='Robot namespace for relay (e.g. tb4_1, tb4_2)'
        ),

        Node(
            package='muro_bridge',
            executable='relay',
            name='relay',
            output='screen',
            parameters=[{
                'namespace': namespace
            }]
        )
    ])
