from dataclasses import dataclass
from enum import Enum
import pathlib
from typing import Any, Optional, Dict, Tuple
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


STATUS_TO_COLOR = {
    Status.NOMINAL: "green",
    Status.WARNING: "yellow",
    Status.ERROR: "red",
    Status.NO_HB: "yellow",
    Status.STARTUP: "yellow",
}


def split_nickname(nickname: str, default_id="N/A") -> Tuple[str, str]:
    """Get robot_id if set."""
    parts = nickname.split("/")
    if len(parts) == 2:
        return parts[1], parts[0]

    return parts[0], default_id


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

    @classmethod
    def from_msg(cls, msg: NodeInfoMsg):
        status, note = value_to_status(msg.status)
        note = msg.notes if note is None else msg.notes + f" ({note})"
        return cls(
            msg.last_heartbeat,
            node_name=msg.node_name,
            status=status,
            notes=note,
            required=msg.required,
        )

    def update(self, other, time_ns):
        self.status = other.status
        self.notes = other.notes
        self.node_name = other.node_name
        self.last_heartbeat = time_ns

    def seconds_since_heartbeat(self, curr_time_s: float):
        return curr_time_s - (self.last_heartbeat * 1.0e-9)

    def to_msg(self):
        msg = NodeInfoMsg()
        msg.last_heartbeat = self.last_heartbeat
        msg.node_name = self.node_name
        msg.status = self.status.value
        msg.notes = self.notes
        msg.required = self.required
        return msg


def _color_entry(entry, color):
    return f"[{color}]{entry}[/{color}]"


@dataclass
class DiagnosticTable:
    rows: Dict[str, TrackedNodeInfo]
    max_no_heartbeat_s: float

    def tick(self, curr_time_s: float):
        """Update the table."""
        for _, info in self.rows.items():
            if info.seconds_since_heartbeat(curr_time_s) > self.max_no_heartbeat_s:
                info.status = Status.NO_HB

    def dump_table(self, curr_time_s):
        table = Table(show_header=True, header_style="bold magenta")
        table.add_column("Nickname")
        table.add_column("Robot ID")
        table.add_column("ROS Name")
        table.add_column("Status")
        table.add_column("Notes")
        table.add_column("Time Since Heartbeat")

        keys = sorted(self.rows.keys())
        for k in keys:
            nn, rid = split_nickname(k)
            info = self.rows[k]
            row = (
                nn,
                rid,
                info.node_name,
                info.status,
                info.notes,
                rf"{info.seconds_since_heartbeat(curr_time_s):.3f} \[s]",
            )

            color = STATUS_TO_COLOR.get(info.status, "red")
            if info.status == Status.NO_HB and info.required:
                color = "red"

            row = tuple(_color_entry(x, color) for x in row)
            table.add_row(*row)

        return table


def print_table(console, table: Table, clean=True):
    """Display a table to the console."""
    if clean:
        sys.stdout.write(chr(27) + "[2J")
        sys.stdout.write("\033[0;0H")

    console.print(table)
