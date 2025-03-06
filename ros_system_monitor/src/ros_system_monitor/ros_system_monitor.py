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
from typing import List

import argparse

CLEAN_TABLE_PRINTS = True


@dataclass
class TrackedNodeInfo:
    node_info: NodeInfo
    last_heartbeat: int


@dataclass
class DiagnosticTable:
    rows: Dict[str, TrackedNodeInfo]


def info_to_row(info: TrackedNodeInfo):
    c1 = info.node_info.nickname
    c2 = info.node_info.node_name
    c3 = info.node_info.status
    c4 = info.node_info.notes

    t_now = time.time()
    dt = t_now - info.last_heartbeat / 1e9
    c5 = dt

    max_heartbeat_interval = 10
    status = info.node_info.status
    if dt > max_heartbeat_interval:
        status = Status.NO_HB

    status_to_color = {
        Status.NOMINAL: "green",
        Status.WARNING: "yellow",
        Status.ERROR: "red",
        Status.NO_HB: "yellow",
        Status.STARTUP: "yellow",
    }

    color = status_to_color[status]

    row = (c1, c2, c3, c4, c5)
    return tuple(map(lambda x: f"[{color}]{x}[/{color}]", row))


def generate_table(diagnostics: DiagnosticTable):
    table = Table(show_header=True, header_style="bold magenta")
    table.add_column("Nickname")
    table.add_column("ROS Name")
    table.add_column("Status")
    table.add_column("Notes")
    table.add_column("Time Since Heartbeat")

    keys = sorted(diagnostics.rows.keys())
    for k in keys:
        row = diagnostics.rows[k]
        table.add_row(*info_to_row(row))
    return table


def print_table(console, table: Table):
    if CLEAN_TABLE_PRINTS:
        sys.stdout.write(chr(27) + "[2J")
        sys.stdout.write("\033[0;0H")
    console.print(table)


@dataclass
class SystemMonitorConfig(Config):
    nodes_to_track: List[str] = field(default_factory=lambda: [])
    external_monitors: Any = config_field("external_monitor")
    # external_monitors: list = field(default_factory=lambda: [])

    @classmethod
    def load(cls, path: str):
        return Config.load(SystemMonitorConfig, path)


class SystemMonitor(Node):
    def __init__(self, config_path):
        super().__init__("ros_system_monitor")

        self.console = Console()
        self.info_lock = threading.Lock()
        self.external_info_subscribers = []

        discover_plugins("rsm_")
        # plugins = discover_plugins("rsm_")
        # self.get_logger().info(f"plugins: {plugins}")
        self.config = SystemMonitorConfig.load(config_path)

        tracked_infos = {}
        for n in self.config.nodes_to_track:
            info = NodeInfo(
                nickname=n, node_name="???", status=Status.NO_HB, notes="Not yet seen"
            )
            tracked_info = TrackedNodeInfo(info, 0)
            tracked_infos[n] = tracked_info
        self.diagnostics = DiagnosticTable(tracked_infos)

        # self.external_monitors = [m.create() for m in self.config.external_monitors]
        self.external_monitors = [self.config.external_monitors.create()]
        for e in self.external_monitors:
            e.register_callbacks(self)

        self.subscriber = self.create_subscription(
            NodeInfoMsg, "~/node_diagnostic_collector", self.callback, 10
        )

        timer_period_s = 0.5
        self.timer = self.create_timer(timer_period_s, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        with self.info_lock:
            table = generate_table(self.diagnostics)
        print_table(self.console, table)

    def callback(self, msg):
        info = NodeInfo.from_ros(msg)
        self.update_node_info(info)

    def add_subscribers(self, subscribers):
        self.external_info_subscribers += subscribers

    def update_node_info(self, info):
        tracked_info = TrackedNodeInfo(info, time.time() * 1e9)

        with self.info_lock:
            tracked_nodes = self.diagnostics.rows.keys()
            if info.nickname in tracked_nodes:
                self.diagnostics.rows[info.nickname] = tracked_info
            else:
                self.get_logger().error(
                    f"Received diagnostics from untracked node: {info.nickname} ({info.node_name})"
                )


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--param_file", type=str)
    args, _ = parser.parse_known_args()
    node = SystemMonitor(args.param_file)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
