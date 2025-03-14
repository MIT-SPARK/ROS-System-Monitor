from dataclasses import dataclass
from enum import Enum
import pathlib
from typing import Any, Optional, Dict
import time
from ros_system_monitor_msgs.msg import NodeInfoMsg
import spark_config as sc
from rich.table import Table
import sys


class Status(Enum):
    NOMINAL = NodeInfoMsg.NOMINAL
    WARNING = NodeInfoMsg.WARNING
    ERROR = NodeInfoMsg.ERROR
    NO_HB = NodeInfoMsg.NO_HB
    STARTUP = NodeInfoMsg.STARTUP


def str_to_status(status: str) -> Status:
    """Convert a string to a status value."""
    try:
        return Status[status.upper()], None
    except KeyError:
        names = [x.name for x in Status]
        return Status.ERROR, f"Invalid enum name '{status.upper()}'. Available: {names}"


def value_to_status(status: int) -> Status:
    """Convert an integer value to a status value."""
    try:
        return Status(status), None
    except Exception:
        return Status.ERROR, f"Invalid status value: {status}"


def get_monitor_topic(nickname, suffix="status"):
    """Get joined path for namespaced monitor."""
    return str(pathlib.Path("~") / nickname / suffix)


@dataclass
class TrackedNodeConfig(sc.Config):
    """
    Configuration specifying a node to track.

    Attributes:
        required: Report an error if the node is missing
        external_monitor: Custom monitor implementation for the node
    """

    required: bool = True
    external_monitor: Any = sc.config_field("external_monitor", required=False)


@dataclass
class TrackedNodeInfo:
    """
    Status information about a tracked node."""

    last_heartbeat: int
    node_name: str = "???"
    status: Status = Status.STARTUP
    notes: str = "Not yet seen"
    external_monitor: Optional[Any] = None
    required: bool = True

    @classmethod
    def from_config(cls, config: TrackedNodeConfig, nickname: str, timestamp_ns: int):
        """Construct the tracking information for the node from a config."""
        return cls(
            timestamp_ns,
            external_monitor=config.external_monitor.create(nickname),
            required=config.required,
        )


@dataclass
class DiagnosticTable:
    rows: Dict[str, TrackedNodeInfo]

    @staticmethod
    def info_to_row(
        nickname: str,
        info: TrackedNodeInfo,
        curr_time_s: float,
        max_no_heartbeat_s: float = 10.0,
    ):
        c1 = nickname
        c2 = info.node_name
        c3 = info.status
        c4 = info.notes

        dt = curr_time_s - info.last_heartbeat / 1e9
        c5 = rf"{dt:.3f} \[s]"

        if dt > max_no_heartbeat_s:
            info.status = Status.NO_HB

        status_to_color = {
            Status.NOMINAL: "green",
            Status.WARNING: "yellow",
            Status.ERROR: "red",
            Status.NO_HB: "yellow",
            Status.STARTUP: "yellow",
        }

        if info.status == Status.NO_HB and info.required:
            color = "red"
        else:
            color = status_to_color[info.status]

        row = (c1, c2, c3, c4, c5)
        return tuple(map(lambda x: f"[{color}]{x}[/{color}]", row))

    def dump_table(self, max_no_heartbeat_s):
        table = Table(show_header=True, header_style="bold magenta")
        table.add_column("Nickname")
        table.add_column("ROS Name")
        table.add_column("Status")
        table.add_column("Notes")
        table.add_column("Time Since Heartbeat")

        t_now = time.time()
        keys = sorted(self.rows.keys())
        for k in keys:
            table.add_row(*self.info_to_row(k, self.rows[k], t_now, max_no_heartbeat_s))

        return table


def print_table(console, table: Table, clean=True):
    """Display a table to the console."""
    if clean:
        sys.stdout.write(chr(27) + "[2J")
        sys.stdout.write("\033[0;0H")

    console.print(table)
