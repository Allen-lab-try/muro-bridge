from launch import LaunchDescription
from launch_ros.actions import Node
import json

def generate_launch_description():

    offsets = {
        "tb4_1": [0.0, 0.0, 0.0],
        "tb4_2": [2.0, 0.0, 0.0],
        "tb4_3": [2.0, 2.0, 0.0],
        "tb4_4": [2.0, -2.0, 0.0],
        "tb4_5": [-2.0, 0.0, 0.0],
        "tb4_6": [-2.0, 2.0, 0.0],
        "tb4_7": [-2.0, -2.0, 0.0],
    }

    return LaunchDescription([
        Node(
            package='muro_bridge',
            executable='tf_aggregator',
            name='central_tf_hub',
            output='screen',
            parameters=[{
                'auto_discovery': True,
                'offsets': json.dumps(offsets),
                'rebroadcast_period_sec': 2.0,
                'max_rebroadcasts': 3,
            }]
        )
    ])
