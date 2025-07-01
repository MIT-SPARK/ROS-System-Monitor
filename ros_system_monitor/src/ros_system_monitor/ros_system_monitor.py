import rclpy
from rclpy.node import Node
from dataclasses import dataclass, field
from rich.console import Console
import time

from ros_system_monitor_msgs.msg import NodeInfoMsg, StatusTable

from ros_system_monitor.diagnostics import (
    DiagnosticTable,
    TrackedNodeConfig,
    TrackedNodeInfo,
    print_table,
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


def _get_nickname(info_name, robot_name):
    return info_name if robot_name is None else f"{robot_name}/{info_name}"


class SystemMonitor(Node):
    def __init__(self, config_path, name=None, should_print=True):
        super().__init__("ros_system_monitor")

        self.console = Console()
        self.info_lock = threading.Lock()
        self._start_time_ns = self.get_clock().now().nanoseconds
        self._should_print = should_print

        discover_plugins("rsm_")
        self.config = SystemMonitorConfig.load(config_path)
        self.name = name

        entries = {}
        for nickname, info_config in self.config.nodes_to_track.items():
            nickname = _get_nickname(nickname, self.name)
            entry = TrackedNodeInfo.from_config(
                info_config, nickname, self._start_time_ns
            )
            if entry.external_monitor is not None:
                entry.external_monitor.register_monitor(self)

            entries[nickname] = entry

        self.diagnostics = DiagnosticTable(
            entries, self.config.max_heartbeat_interval_s
        )
        self.diagnostics_sub = self.create_subscription(
            NodeInfoMsg, "~/node_diagnostic_collector", self._info_callback, 10
        )
        self.forward_sub = self.create_subscription(
            StatusTable, "~/table_in", self._table_callback, 10
        )

        self.forward_pub = None
        if self.name is not None:
            self.forward_pub = self.create_publisher(StatusTable, "~/table_out", 10)

        self.timer = self.create_timer(self.config.timer_period_s, self._timer_callback)

    def add_monitor(self, nickname: str, config: TrackedNodeConfig):
        tracked_info = TrackedNodeInfo.from_config(config, self._start_time_ns)
        if tracked_info.external_monitor is not None:
            tracked_info.register_callbacks(self)

        self.diagnostics.rows[nickname] = tracked_info

    def update_node_info(self, nickname, info: TrackedNodeInfo):
        time_ns = int(time.time() * 1e9)
        with self.info_lock:
            if nickname not in self.diagnostics.rows:
                info_str = f"'{nickname}' ('{info.node_name}')"
                self.get_logger().error(f"Got untracked node status: {info_str}")
                return

            self.diagnostics.rows[nickname].update(info, time_ns)

    def _timer_callback(self):
        t_now = time.time()
        with self.info_lock:
            self.diagnostics.tick(t_now)
            table = self.diagnostics.dump_table(t_now)
            if self.forward_pub is not None:
                self._forward_table()

        if self._should_print:
            print_table(self.console, table, clean=self.config.clean_prints)

    def _info_callback(self, msg):
        nn = _get_nickname(msg.nickname, self.name)
        self.update_node_info(nn, TrackedNodeInfo.from_msg(msg))

    def _forward_table(self):
        msg = StatusTable()
        msg.monitor_name = self.name
        for nn, row in self.diagnostics.rows.items():
            row_msg = row.to_msg()
            row_msg.nickname = nn
            msg.status.append(row_msg)

        self.forward_pub.publish(msg)

    def _table_callback(self, msg):
        with self.info_lock:
            for status in msg.status:
                self.diagnostics.rows[status.nickname] = TrackedNodeInfo.from_msg(
                    status
                )


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--param_file", type=str)
    parser.add_argument("--name", type=str, default=None)
    parser.add_argument("--print", action=argparse.BooleanOptionalAction, default=True)
    args, _ = parser.parse_known_args()
    node = SystemMonitor(args.param_file, name=args.name, should_print=args.print)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
