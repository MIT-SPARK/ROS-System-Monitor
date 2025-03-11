from dataclasses import dataclass
from enum import Enum
from ros_system_monitor_msgs.msg import NodeInfoMsg


class Status(Enum):
    NOMINAL = 1
    WARNING = 2
    ERROR = 3
    NO_HB = 4
    STARTUP = 5


@dataclass
class NodeInfo:
    nickname: str
    node_name: str = "???"
    status: Status = Status.STARTUP
    notes: str = "Not yet seen"

    @classmethod
    def from_ros(cls, msg):
        return cls(
            nickname=msg.nickname,
            node_name=msg.node_name,
            status=Status(msg.status),
            notes=msg.notes,
        )

    def to_ros(self):
        msg = NodeInfoMsg()
        msg.nickname = self.nickname
        msg.node_name = self.node_name
        msg.status = self.status
        msg.notes = self.notes
        return msg
