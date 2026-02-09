#!/usr/bin/env python3
# ============================================================
# Central TF Hub — Aggregator Node
# ============================================================

import re
import json
import math
from typing import Dict, Set, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


NS_TF_PATTERN = re.compile(r"^/([^/]+)/tf$")
NS_TF_STATIC_PATTERN = re.compile(r"^/([^/]+)/tf_static$")


class CentralTFHub(Node):
    def __init__(self):
        super().__init__('central_tf_hub')

        # ---------------- QoS ----------------
        self.qos_tf = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.qos_tf_static = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        # ---------------- Parameters ----------------
        self.declare_parameter('robots', [])

        # >>> 7-robot baseline offsets
        self.declare_parameter('offsets', json.dumps({
            "tb4_1": [0.0, 0.0, 0.0],
            "tb4_2": [2.0, 0.0, 0.0],
            "tb4_3": [2.0, 2.0, 0.0],
            "tb4_4": [2.0, -2.0, 0.0],
            "tb4_5": [-2.0, 0.0, 0.0],
            "tb4_6": [-2.0, 2.0, 0.0],
            "tb4_7": [-2.0, -2.0, 0.0],
        }))

        self.declare_parameter('auto_discovery', True)
        self.declare_parameter('rebroadcast_period_sec', 2.0)
        self.declare_parameter('max_rebroadcasts', 3)

        robots_param = self.get_parameter('robots').get_parameter_value().string_array_value
        self.config_robots: List[str] = list(robots_param) if robots_param else []

        self.auto_discovery = self.get_parameter('auto_discovery').value
        self.rebroadcast_period = float(self.get_parameter('rebroadcast_period_sec').value)
        self.max_rebroadcasts = int(self.get_parameter('max_rebroadcasts').value)

        offsets_str = self.get_parameter('offsets').get_parameter_value().string_value
        try:
            self.offsets: Dict[str, List[float]] = json.loads(offsets_str)
        except Exception as e:
            self.get_logger().warn(f"offsets JSON parse failed: {e}")
            self.offsets = {}

        # ---------------- Publishers ----------------
        self.pub_tf = self.create_publisher(TFMessage, '/tf', self.qos_tf)
        self.pub_tf_static = self.create_publisher(TFMessage, '/tf_static', self.qos_tf_static)
        self.static_br = StaticTransformBroadcaster(self)

        # ---------------- State ----------------
        self.known_robots: Set[str] = set()
        self.tf_subs = {}
        self.tf_static_subs = {}

        self.world_map_transforms: Dict[str, TransformStamped] = {}
        self.rebroadcast_attempts = 0

        # ---------------- Startup ----------------
        self._prepare_world_to_map_transforms()

        for ns in self.config_robots:
            self._ensure_robot_subscriptions(ns)

        if self.auto_discovery:
            self.create_timer(1.5, self._discover_robots_from_topics)

        self.create_timer(self.rebroadcast_period, self._rebroadcast_static_once)

        self.get_logger().info("✅ Central TF Hub started")
        self.get_logger().info(f"   auto_discovery={self.auto_discovery}")
        self.get_logger().info(f"   offsets={list(self.offsets.keys())}")

    # ---------------- world → map ----------------
    def _prepare_world_to_map_transforms(self):
        now = self.get_clock().now().to_msg()

        for ns, off in self.offsets.items():
            try:
                ox, oy, oyaw_deg = off
            except Exception:
                continue

            yaw = math.radians(oyaw_deg)

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'world'
            t.child_frame_id = f'{ns}/map'
            t.transform.translation.x = ox
            t.transform.translation.y = oy
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)

            self.world_map_transforms[ns] = t

        self._publish_world_to_map_once("🌍 Initial")

    def _publish_world_to_map_once(self, prefix="🔁"):
        if not self.world_map_transforms:
            return

        now = self.get_clock().now().to_msg()
        for t in self.world_map_transforms.values():
            t.header.stamp = now

        self.static_br.sendTransform(list(self.world_map_transforms.values()))
        self.get_logger().info(f"{prefix} world→map published")

    def _rebroadcast_static_once(self):
        if self.max_rebroadcasts != -1 and self.rebroadcast_attempts >= self.max_rebroadcasts:
            return
        self.rebroadcast_attempts += 1
        self._publish_world_to_map_once(f"🔁 Attempt {self.rebroadcast_attempts}")

    # ---------------- Discovery ----------------
    def _discover_robots_from_topics(self):
        for name, _ in self.get_topic_names_and_types():
            for pat in (NS_TF_PATTERN, NS_TF_STATIC_PATTERN):
                m = pat.match(name)
                if m:
                    self._ensure_robot_subscriptions(m.group(1))

    # ---------------- Subscriptions ----------------
    def _ensure_robot_subscriptions(self, ns: str):
        if ns in self.known_robots:
            return

        self.tf_subs[ns] = self.create_subscription(
            TFMessage, f'/{ns}/tf',
            lambda msg: self.pub_tf.publish(msg),
            self.qos_tf
        )

        self.tf_static_subs[ns] = self.create_subscription(
            TFMessage, f'/{ns}/tf_static',
            lambda msg: self.pub_tf_static.publish(msg),
            self.qos_tf_static
        )

        self.known_robots.add(ns)
        self.get_logger().info(f"🛰️ Subscribed TF streams: {ns}")

        if ns in self.world_map_transforms:
            self._publish_world_to_map_once(f"🌍 New robot {ns}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CentralTFHub())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
