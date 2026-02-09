from launch_ros.actions import Node
import os


# ============================================================
# Internal helper
# ============================================================

def _create_domain_bridge(*, config_file: str, node_name: str):
    """
    Create a single domain_bridge node with a given YAML config.
    """
    return Node(
        package='domain_bridge',
        executable='domain_bridge',
        name=node_name,
        arguments=[config_file],
        output='screen',
    )


# ============================================================
# Robot (domain = tb4_X) -> Aggregator (domain 99)
# ============================================================

def robot_to_aggregator(*, robot_ids, config_dir):
    """
    Create bridges from robot DDS domains to aggregator domain (99).

    Config layout:
      config/bridge_robot_to_99/
        ├── bridge_tb4_1.yaml
        ├── bridge_tb4_2.yaml
        └── ...
    """
    nodes = []

    robot_cfg_dir = os.path.join(config_dir, 'bridge_robot_to_99')

    for rid in robot_ids:
        cfg = os.path.join(
            robot_cfg_dir,
            f'bridge_tb4_{rid}.yaml'
        )

        nodes.append(
            _create_domain_bridge(
                config_file=cfg,
                node_name=f'bridge_tb4_{rid}_to_99',
            )
        )

    return nodes


# ============================================================
# Aggregator (domain 99) -> Global (domain 100)
# ============================================================

def aggregator_to_global(*, config_dir):
    """
    Create the singleton bridge from aggregator domain (99) to global domain (100).

    Config layout:
      config/bridge_99_to_100/bridge_99_to_100.yaml
    """
    cfg = os.path.join(
        config_dir,
        'bridge_99_to_100',
        'bridge_99_to_100.yaml'
    )

    return _create_domain_bridge(
        config_file=cfg,
        node_name='bridge_99_to_100',
    )


# ============================================================
# Global (domain 100) -> Robot (shared config)
# ============================================================

def global_to_robot(*, config_dir):
    """
    Create the bridge from global domain (100) back to robot domains.

    Currently implemented as a single shared bridge.

    Config layout:
      config/bridge_global_to_robot/bridge_global_to_robot.yaml
    """
    cfg = os.path.join(
        config_dir,
        'bridge_global_to_robot',
        'bridge_global_to_robot.yaml'
    )

    return _create_domain_bridge(
        config_file=cfg,
        node_name='bridge_global_to_robot',
    )


