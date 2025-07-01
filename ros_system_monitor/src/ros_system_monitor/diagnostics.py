from dataclasses import dataclass
from enum import Enum
import pathlib
from typing import Any, Optional, Tuple
from ros_system_monitor_msgs.msg import NodeInfoMsg
import spark_config as sc
from rich.table import Table
import copy
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

    nickname: str
    last_heartbeat: int
    node_name: str = "???"
    status: Status = Status.STARTUP
    notes: str = "Not yet seen"
    required: bool = True
    external_monitor: Optional[Any] = None

    @classmethod
    def from_config(cls, config: TrackedNodeConfig, nickname: str, timestamp_ns: int):
        """Construct the tracking information for the node from a config."""
        return cls(
            nickname,
            timestamp_ns,
            external_monitor=config.external_monitor.create(nickname),
            required=config.required,
        )

    @classmethod
    def from_msg(cls, msg: NodeInfoMsg):
        status, note = value_to_status(msg.status)
        note = msg.notes if note is None else msg.notes + f" ({note})"
        return cls(
            msg.nickname,
            msg.last_heartbeat,
            node_name=msg.node_name,
            status=status,
            notes=note,
            required=msg.required,
        )

    def update(self, other, time_ns: Optional[int] = None):
        if self.nickname != other.nickname:
            raise ValueError(
                f"Nicknames do not match: '{self.nickname}' != '{other.nickname}'"
            )

        self.status = other.status
        self.notes = other.notes
        self.node_name = other.node_name
        self.last_heartbeat = time_ns or other.last_heartbeat

    def seconds_since_heartbeat(self, curr_time_s: float):
        return curr_time_s - (self.last_heartbeat * 1.0e-9)

    def to_msg(self):
        msg = NodeInfoMsg()
        msg.nickname = self.nickname
        msg.last_heartbeat = self.last_heartbeat
        msg.node_name = self.node_name
        msg.status = self.status.value
        msg.notes = self.notes
        msg.required = self.required
        return msg


def _color_entry(entry, color):
    return f"[{color}]{entry}[/{color}]"


class DiagnosticTable:
    def __init__(self, max_no_heartbeat_s: float):
        self.rows = {}
        self.max_no_heartbeat_s = max_no_heartbeat_s

    def add(self, info: TrackedNodeInfo):
        self.rows[info.nickname] = info

    def update(self, info: TrackedNodeInfo, time_ns: Optional[int] = None):
        if info.nickname not in self.rows:
            info_str = f"'{info.nickname}' ('{info.node_name}')"
            self.get_logger().error(f"Got untracked node status: {info_str}")
            return

        self.rows[info.nickname].update(info, time_ns)

    def tick(self, curr_time_s: float):
        """Update the table."""
        for _, info in self.rows.items():
            if info.seconds_since_heartbeat(curr_time_s) > self.max_no_heartbeat_s:
                info.status = Status.NO_HB

    def dump(self):
        return [copy.deepcopy(x) for _, x in self.rows.items()]

    @classmethod
    def from_msg(cls, msg):
        table = cls(msg.max_no_heartbeat_s)
        for info in msg.status:
            table.add(TrackedNodeInfo.from_msg(info))

        return table


def print_table(console, rows: list[TrackedNodeInfo], curr_time_s, clean=True):
    """Display a table to the console."""
    if clean:
        sys.stdout.write(chr(27) + "[2J")
        sys.stdout.write("\033[0;0H")

    table = Table(show_header=True, header_style="bold magenta")
    table.add_column("Nickname")
    table.add_column("Robot ID")
    table.add_column("ROS Name")
    table.add_column("Status")
    table.add_column("Notes")
    table.add_column("Time Since Heartbeat")

    rows = sorted(rows, key=lambda x: x.nickname)
    for info in rows:
        nickname, robot_id = split_nickname(info.nickname)
        row = (
            nickname,
            robot_id,
            info.node_name,
            info.status,
            info.notes,
            rf"{info.seconds_since_heartbeat(curr_time_s):.3f} \[s]",
        )

        color = STATUS_TO_COLOR.get(info.status, "red")
        if info.status == Status.NO_HB and info.required:
            color = "red"

        table.add_row(*tuple(_color_entry(x, color) for x in row))

    console.print(table)
