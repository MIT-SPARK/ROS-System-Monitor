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
import pathlib
import threading

from spark_config import Config, discover_plugins
from typing import Dict, Optional

import argparse


@dataclass
class SystemMonitorConfig(Config):
    """
    Configuration for the system monitor.

    Attributes:
        nodes_to_track: Declaration of the nodes that the system monitor should display
        max_no_heartbeat_s: Max time without receiving a heartbeat from a node
        timer_period_s: Refresh rate of the system monitor
    """

    nodes_to_track: Dict[str, TrackedNodeConfig] = field(default_factory=dict)
    max_no_heartbeat_s: float = 10.0
    timer_period_s: float = 0.5
    clean_prints: bool = True

    @classmethod
    def load(cls, path: Optional[str]):
        if path is None:
            return cls()

        path = pathlib.Path(path).expanduser().absolute()
        if not path.exists():
            return cls()

        return Config.load(SystemMonitorConfig, path)


def _get_nickname(info_name, robot_name):
    return info_name if robot_name is None else f"{robot_name}/{info_name}"


class SystemMonitor(Node):
    def __init__(
        self, config_path, name=None, should_print=True, max_no_heartbeat_s=None
    ):
        super().__init__("ros_system_monitor")

        self.console = Console()
        self.info_lock = threading.Lock()
        self._start_time_ns = self.get_clock().now().nanoseconds
        self._should_print = should_print

        discover_plugins("rsm_")
        self.config = SystemMonitorConfig.load(config_path)
        self.get_logger().info(f"config: {self.config}")
        self.config.max_no_heartbeat_s = (
            max_no_heartbeat_s or self.config.max_no_heartbeat_s
        )
        self.name = name

        self.diagnostics = DiagnosticTable(self.config.max_no_heartbeat_s)
        for nickname, config in self.config.nodes_to_track.items():
            nickname = _get_nickname(nickname, self.name)

            entry = TrackedNodeInfo.from_config(config, nickname, self._start_time_ns)
            entry.external_monitor = config.external_monitor.create(nickname, self.name)
            if entry.external_monitor is not None:
                entry.external_monitor.register_monitor(self)

            self.diagnostics.add(entry)

        self.forwarded_diagnostics = {}
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

        self.diagnostics.add(tracked_info)

    def update_node_info(self, info: TrackedNodeInfo):
        with self.info_lock:
            if not self.diagnostics.update(info, int(time.time() * 1e9)):
                info_str = f"'{info.nickname}' ('{info.node_name}')"
                self.get_logger().error(f"Got untracked node status: {info_str}")

    def _timer_callback(self):
        t_now = time.time()
        with self.info_lock:
            self.diagnostics.tick(t_now)
            rows = self.diagnostics.dump()
            for _, table in self.forwarded_diagnostics.items():
                table.tick(t_now)
                rows += table.dump()

        if self.forward_pub is not None:
            self._forward_table(rows)

        if self._should_print:
            print_table(self.console, rows, t_now, clean=self.config.clean_prints)

    def _info_callback(self, msg):
        # TODO(nathan) this is still clunky
        nn = _get_nickname(msg.nickname, self.name)
        info = TrackedNodeInfo.from_msg(msg)
        info.nickname = nn
        self.update_node_info(info)

    def _forward_table(self, rows):
        msg = StatusTable()
        msg.monitor_name = self.name
        msg.max_no_heartbeat_s = self.config.max_no_heartbeat_s
        for row in rows:
            msg.status.append(row.to_msg())

        self.forward_pub.publish(msg)

    def _table_callback(self, msg):
        with self.info_lock:
            table_name = msg.monitor_name
            if table_name not in self.forwarded_diagnostics:
                self.forwarded_diagnostics[table_name] = DiagnosticTable.from_msg(msg)

            for info_msg in msg.status:
                info = TrackedNodeInfo.from_msg(info_msg)
                self.forwarded_diagnostics[table_name].update(info)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--param_file", type=str)
    parser.add_argument("--name", type=str, default=None)
    parser.add_argument("--print", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max_no_heartbeat_s", type=float, default=None)
    args, _ = parser.parse_known_args()
    node = SystemMonitor(
        args.param_file,
        name=args.name,
        should_print=args.print,
        max_no_heartbeat_s=args.max_no_heartbeat_s,
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
