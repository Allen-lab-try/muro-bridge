#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import String

# Optional import for Nav2 incremental map updates
try:
    from nav2_msgs.msg import MapUpdate
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False


class FullRelay(Node):
    """
    Full topic + TF relay node.
    Relays global topics into a single robot namespace (e.g. tb4_1),
    while prefixing all TF frame_ids correctly.
    """

    def __init__(self):
        super().__init__('relay_full')

        # -------------------------------
        # Namespace parameter
        # -------------------------------
        self.declare_parameter('namespace', 'tb4_1')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        self.get_logger().info(f'✅ Relay started for namespace: {self.namespace}')

        # -------------------------------
        # QoS Profiles
        # -------------------------------
        qos_volatile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        qos_transient = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        # -------------------------------
        # Static TF cache (for late joiners)
        # -------------------------------
        self._static_cache = []
        self._static_cached_once = False
        self._rebroadcast_attempts = 0

        # -------------------------------
        # TF
        # -------------------------------
        self.sub_tf = self.create_subscription(
            TFMessage, '/tf', self.tf_cb, qos_volatile
        )
        self.pub_tf = self.create_publisher(
            TFMessage, f'/{self.namespace}/tf', qos_volatile
        )

        self.sub_tf_static = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_cb, qos_transient
        )
        self.pub_tf_static = self.create_publisher(
            TFMessage, f'/{self.namespace}/tf_static', qos_transient
        )

        # -------------------------------
        # Scan
        # -------------------------------
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_volatile
        )
        self.pub_scan = self.create_publisher(
            LaserScan, f'/{self.namespace}/scan', qos_volatile
        )

        # -------------------------------
        # Odometry
        # -------------------------------
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_cb, qos_volatile
        )
        self.pub_odom = self.create_publisher(
            Odometry, f'/{self.namespace}/odom', qos_volatile
        )

        # -------------------------------
        # Robot description & joints
        # -------------------------------
        self.sub_urdf = self.create_subscription(
            String, '/robot_description', self.urdf_cb, qos_transient
        )
        self.pub_urdf = self.create_publisher(
            String, f'/{self.namespace}/robot_description', qos_transient
        )

        self.sub_joint = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, qos_volatile
        )
        self.pub_joint = self.create_publisher(
            JointState, f'/{self.namespace}/joint_states', qos_volatile
        )

        # -------------------------------
        # Map
        # -------------------------------
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, qos_transient
        )
        self.pub_map = self.create_publisher(
            OccupancyGrid, f'/{self.namespace}/map', qos_transient
        )

        self.sub_map_metadata = self.create_subscription(
            MapMetaData, '/map_metadata', self.map_meta_cb, qos_transient
        )
        self.pub_map_metadata = self.create_publisher(
            MapMetaData, f'/{self.namespace}/map_metadata', qos_transient
        )

        if NAV2_AVAILABLE:
            self.sub_map_updates = self.create_subscription(
                MapUpdate, '/map_updates', self.map_update_cb, qos_transient
            )
            self.pub_map_updates = self.create_publisher(
                MapUpdate, f'/{self.namespace}/map_updates', qos_transient
            )
        else:
            self.get_logger().warn(
                '⚠️ nav2_msgs not available, skipping /map_updates relay'
            )

        # -------------------------------
        # Delayed static TF rebroadcast
        # -------------------------------
        self._rebroadcast_timer = self.create_timer(
            2.0, self._rebroadcast_static_once
        )

        self.get_logger().info(
            f'📡 Relaying to /{self.namespace}/* : '
            f'tf, tf_static, scan, odom, map, map_metadata, robot_description, joint_states'
        )

    # =========================================================
    # Helper
    # =========================================================
    def _add_prefix_if_needed(self, frame_id: str) -> str:
        if not frame_id:
            return f'{self.namespace}/unknown'

        fid = frame_id.lstrip('/')
        ns_prefix = f'{self.namespace}/'

        if fid.startswith(ns_prefix):
            return fid

        # Strip TurtleBot4 default prefix if present
        if fid.startswith('turtlebot4/'):
            fid = fid[len('turtlebot4/'):]

        return ns_prefix + fid

    def _prefix_tf_msg(self, msg: TFMessage) -> TFMessage:
        out = TFMessage()
        for t in msg.transforms:
            nt = TransformStamped()
            nt.header = t.header
            nt.header.frame_id = self._add_prefix_if_needed(t.header.frame_id)
            nt.child_frame_id = self._add_prefix_if_needed(t.child_frame_id)
            nt.transform = t.transform
            out.transforms.append(nt)
        return out

    # =========================================================
    # Callbacks
    # =========================================================
    def tf_cb(self, msg: TFMessage):
        self.pub_tf.publish(self._prefix_tf_msg(msg))

    def tf_static_cb(self, msg: TFMessage):
        converted = self._prefix_tf_msg(msg)
        self.pub_tf_static.publish(converted)

        if converted.transforms:
            self._static_cache.extend(converted.transforms)
            self._static_cached_once = True

    def scan_cb(self, msg: LaserScan):
        msg.header.frame_id = self._add_prefix_if_needed(msg.header.frame_id)
        self.pub_scan.publish(msg)

    def odom_cb(self, msg: Odometry):
        msg.header.frame_id = self._add_prefix_if_needed(msg.header.frame_id)
        if msg.child_frame_id:
            msg.child_frame_id = self._add_prefix_if_needed(msg.child_frame_id)
        self.pub_odom.publish(msg)

    def urdf_cb(self, msg: String):
        self.pub_urdf.publish(msg)

    def joint_cb(self, msg: JointState):
        self.pub_joint.publish(msg)

    def map_cb(self, msg: OccupancyGrid):
        msg.header.frame_id = self._add_prefix_if_needed(msg.header.frame_id)
        self.pub_map.publish(msg)

    def map_meta_cb(self, msg: MapMetaData):
        self.pub_map_metadata.publish(msg)

    def map_update_cb(self, msg):
        msg.header.frame_id = self._add_prefix_if_needed(msg.header.frame_id)
        self.pub_map_updates.publish(msg)

    # =========================================================
    # Static TF rebroadcast (late joiner support)
    # =========================================================
    def _rebroadcast_static_once(self):
        if self._rebroadcast_attempts >= 3:
            self._rebroadcast_timer.cancel()
            return

        self._rebroadcast_attempts += 1

        if not self._static_cached_once or not self._static_cache:
            self.get_logger().warn(
                f'⏳ Static TF not ready, retry {self._rebroadcast_attempts}/3'
            )
            return

        out = TFMessage()
        out.transforms.extend(self._static_cache)
        self.pub_tf_static.publish(out)

        self.get_logger().info(
            f'🔁 Rebroadcasted {len(self._static_cache)} static TFs'
        )
        self._rebroadcast_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = FullRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
