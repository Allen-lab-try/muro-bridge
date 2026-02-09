import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from muro_bridge.multi_bridge import (
    robot_to_aggregator,
    aggregator_to_global,
    global_to_robot,
)


def generate_launch_description():
    """
    Launch all domain bridges for multi-robot simulation.
    """

    config_dir = os.path.join(
        get_package_share_directory('muro_bridge'),
        'config'
    )

    # Robot IDs you have prepared YAMLs for
    robot_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

    nodes = []

    # Robot -> Aggregator (99)
    nodes += robot_to_aggregator(
        robot_ids=robot_ids,
        config_dir=config_dir,
    )

    # Aggregator (99) -> Global (100)
    nodes.append(
        aggregator_to_global(
            config_dir=config_dir,
        )
    )

    # Global (100) -> Robots (shared)
    nodes.append(
        global_to_robot(
            config_dir=config_dir,
        )
    )

    return LaunchDescription(nodes)

