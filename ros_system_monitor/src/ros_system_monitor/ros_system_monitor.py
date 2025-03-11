import rclpy
from rclpy.node import Node
from typing import Dict, Any
from dataclasses import dataclass, field
from rich.console import Console
from rich.table import Table
import sys
import time

from ros_system_monitor_msgs.msg import NodeInfoMsg

from ros_system_monitor.node_info import Status, NodeInfo
import threading

from spark_config import Config, discover_plugins, config_field
from typing import List, Optional

import argparse

CLEAN_TABLE_PRINTS = True


@dataclass
class TrackedNodeConfig(Config):
    nickname: str = ""
    external_monitor: Any = config_field("external_monitor", required=False)
    required: bool = True


@dataclass
class TrackedNodeInfo:
    node_info: NodeInfo
    last_heartbeat: int
    external_monitor: Optional[Any] = None
    required: bool = True

    @classmethod
    def from_config(cls, config: TrackedNodeConfig, timestamp_ns: int):
        """Construct the tracking information for the node from a config."""
        return cls(
            NodeInfo(config.nickname),
            timestamp_ns,
            external_monitor=config.external_monitor.create(),
            required=config.required,
        )


@dataclass
class DiagnosticTable:
    rows: Dict[str, TrackedNodeInfo]


def info_to_row(info: TrackedNodeInfo, max_heartbeat_interval_s: float = 10.0):
    c1 = info.node_info.nickname
    c2 = info.node_info.node_name
    c3 = info.node_info.status
    c4 = info.node_info.notes

    t_now = time.time()
    dt = t_now - info.last_heartbeat / 1e9
    c5 = rf"{dt:.3f} \[s]"

    if dt > max_heartbeat_interval_s:
        info.node_info.status = Status.NO_HB

    status_to_color = {
        Status.NOMINAL: "green",
        Status.WARNING: "yellow",
        Status.ERROR: "red",
        Status.NO_HB: "yellow",
        Status.STARTUP: "yellow",
    }

    status = info.node_info.status
    if status == Status.NO_HB and info.required:
        color = "red"
    else:
        color = status_to_color[status]

    row = (c1, c2, c3, c4, c5)
    return tuple(map(lambda x: f"[{color}]{x}[/{color}]", row))


def generate_table(diagnostics: DiagnosticTable, max_heartbeat_interval_s):
    table = Table(show_header=True, header_style="bold magenta")
    table.add_column("Nickname")
    table.add_column("ROS Name")
    table.add_column("Status")
    table.add_column("Notes")
    table.add_column("Time Since Heartbeat")

    keys = sorted(diagnostics.rows.keys())
    for k in keys:
        row = diagnostics.rows[k]
        table.add_row(*info_to_row(row, max_heartbeat_interval_s))
    return table


def print_table(console, table: Table):
    if CLEAN_TABLE_PRINTS:
        sys.stdout.write(chr(27) + "[2J")
        sys.stdout.write("\033[0;0H")
    console.print(table)


@dataclass
class SystemMonitorConfig(Config):
    nodes_to_track: List[TrackedNodeConfig] = field(default_factory=list)
    max_heartbeat_interval_s: float = 10.0

    @classmethod
    def load(cls, path: str):
        return Config.load(SystemMonitorConfig, path)


class SystemMonitor(Node):
    def __init__(self, config_path):
        super().__init__("ros_system_monitor")

        self.console = Console()
        self.info_lock = threading.Lock()

        discover_plugins("rsm_")
        # plugins = discover_plugins("rsm_")
        # self.get_logger().info(f"plugins: {plugins}")

        self.config = SystemMonitorConfig.load(config_path)
        for c in self.config.nodes_to_track:
            if c.nickname == "":
                self.get_logger().error(f"Unnamed node diagnostics: '{c}'")
                sys.exit(1)

        self._start_time_ns = self.get_clock().now().nanoseconds
        self.diagnostics = DiagnosticTable(
            {
                c.nickname: TrackedNodeInfo.from_config(c, self._start_time_ns)
                for c in self.config.nodes_to_track
            }
        )

        for _, node in self.diagnostics.rows.items():
            if node.external_monitor is not None:
                node.external_monitor.register_callbacks(self)

        self.subscriber = self.create_subscription(
            NodeInfoMsg, "~/node_diagnostic_collector", self.callback, 10
        )

        timer_period_s = 0.5
        self.timer = self.create_timer(timer_period_s, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        with self.info_lock:
            table = generate_table(
                self.diagnostics, self.config.max_heartbeat_interval_s
            )

        print_table(self.console, table)

    def callback(self, msg):
        info = NodeInfo.from_ros(msg)
        self.update_node_info(info)

    def add_node_monitors(self, nodes: List[TrackedNodeConfig]):
        for node in nodes:
            self.diagnostics.rows[node.nickname] = TrackedNodeInfo.from_config(
                node, self._start_time_ns
            )

    def update_node_info(self, info):
        time_ns = time.time() * 1e9
        with self.info_lock:
            tracked_nodes = self.diagnostics.rows.keys()
            if info.nickname not in tracked_nodes:
                self.get_logger().error(
                    f"Received untracked node diagnostics: '{info.nickname}' ('{info.node_name}')"
                )
                return

            self.diagnostics.rows[info.nickname].node_info = info
            self.diagnostics.rows[info.nickname].last_heartbeat = time_ns


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--param_file", type=str)
    args, _ = parser.parse_known_args()
    node = SystemMonitor(args.param_file)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
