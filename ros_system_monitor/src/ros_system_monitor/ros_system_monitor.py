import rclpy
from rclpy.node import Node
from dataclasses import dataclass, field
from rich.console import Console
import time

from ros_system_monitor_msgs.msg import NodeInfoMsg

from ros_system_monitor.diagnostics import (
    DiagnosticTable,
    TrackedNodeConfig,
    TrackedNodeInfo,
    print_table,
    value_to_status,
)
import threading

from spark_config import Config, discover_plugins
from typing import Dict

import argparse


@dataclass
class SystemMonitorConfig(Config):
    """
    Configuration for the system monitor.

    Attributes:
        nodes_to_track: Declaration of the nodes that the system monitor should display
        max_heartbeat_interval_s: Max time without receiving a heartbeat from a node
        timer_period_s: Refresh rate of the system monitor
    """

    nodes_to_track: Dict[str, TrackedNodeConfig] = field(default_factory=dict)
    max_heartbeat_interval_s: float = 10.0
    timer_period_s: float = 0.5
    clean_prints: bool = True

    @classmethod
    def load(cls, path: str):
        return Config.load(SystemMonitorConfig, path)


class SystemMonitor(Node):
    def __init__(self, config_path):
        super().__init__("ros_system_monitor")

        self.console = Console()
        self.info_lock = threading.Lock()
        self._start_time_ns = self.get_clock().now().nanoseconds

        discover_plugins("rsm_")
        self.config = SystemMonitorConfig.load(config_path)
        self.diagnostics = DiagnosticTable(
            {
                n: TrackedNodeInfo.from_config(c, n, self._start_time_ns)
                for n, c in self.config.nodes_to_track.items()
            }
        )

        robots = set([])
        for nickname, node in self.diagnostics.rows.items():
            if node.external_monitor is not None:
                try:
                    node.external_monitor.register_monitor(self)
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to register external monitor '{nickname}': {e}"
                    )

                parts = nickname.split("/")
                if len(parts) == 2:
                    robots.add(parts[0])

        self.subscriber = self.create_subscription(
            NodeInfoMsg, "~/node_diagnostic_collector", self._callback, 10
        )
        for robot in robots:
            self.subscriber = self.create_subscription(
                NodeInfoMsg,
                "~/node_diagnostic_collector/{robot}",
                lambda x: self._callback(x, robot=robot),
                10,
            )

        self.timer = self.create_timer(self.config.timer_period_s, self._timer_callback)

    def add_monitor(self, nickname: str, config: TrackedNodeConfig):
        tracked_info = TrackedNodeInfo.from_config(config, self._start_time_ns)
        if tracked_info.external_monitor is not None:
            tracked_info.register_callbacks(self)

        self.diagnostics.rows[nickname] = tracked_info

    def update_node_info(self, nickname, node_name, status, notes):
        time_ns = int(time.time() * 1e9)
        with self.info_lock:
            if nickname not in self.diagnostics.rows:
                self.get_logger().error(
                    f"Received untracked node diagnostics: '{nickname}' ('{node_name}')"
                )
                return

            self.diagnostics.rows[nickname].status = status
            self.diagnostics.rows[nickname].notes = notes
            self.diagnostics.rows[nickname].node_name = node_name
            self.diagnostics.rows[nickname].last_heartbeat = time_ns

    def _timer_callback(self):
        with self.info_lock:
            table = self.diagnostics.dump_table(self.config.max_heartbeat_interval_s)

        print_table(self.console, table, clean=self.config.clean_prints)

    def _callback(self, msg, robot=None):
        status, note = value_to_status(msg.status)
        note = msg.notes if note is None else msg.notes + f" ({note})"
        nn = msg.nickname if robot is None else f"{msg.robot_id}/{msg.nickname}"
        self.update_node_info(nn, msg.node_name, status, note)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--param_file", type=str)
    args, _ = parser.parse_known_args()
    node = SystemMonitor(args.param_file)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
