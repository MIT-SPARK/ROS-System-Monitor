from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional

from ros_system_monitor.diagnostics import (
    Status,
    TrackedNodeInfo,
    get_monitor_topic,
    str_to_status,
)
import spark_config as sc
from std_msgs.msg import String


class JsonMonitor:
    """External monitor that listens to json status report."""

    def __init__(
        self,
        config: JsonMonitorConfig,
        nickname: str,
        monitor_name: Optional[str] = None,
    ):
        self.nickname = nickname
        self.monitor_name = monitor_name

    @property
    def relative_name(self):
        if self.monitor_name is None:
            return self.nickname

        return self.nickname.split("/")[1]

    def register_monitor(self, monitor):
        self._update_cb = monitor.update_node_info
        topic = get_monitor_topic(self.relative_name)
        self.sub = monitor.create_subscription(String, str(topic), self._callback, 1)

    def _register_error(self, notes, node_name=None):
        self._update_cb(
            TrackedNodeInfo(
                nickname=self.nickname,
                last_heartbeat=-1,
                node_name=node_name or "???",
                status=Status.ERROR,
                notes=notes,
            )
        )

    def _callback(self, msg):
        try:
            contents = json.loads(msg.data)
        except Exception as e:
            self._register_error(f"invalid json: {e} (data='{msg.data}')")
            return

        if "status" not in contents:
            self._register_error(f"status missing: '{contents}'")
            return

        if "nickname" not in contents:
            self._register_error(f"nickname missing: '{contents}'")
            return

        curr_name = contents["nickname"]
        if self.relative_name != curr_name:
            self._register_error(
                f"name mismatch: got '{curr_name} vs. '{self.nickname}'",
                node_name=contents.get("node_name"),
            )
            return

        status, status_str = str_to_status(contents["status"])
        note = contents.get("note", "")
        if status_str is not None:
            note += f" ({status_str})"

        info = TrackedNodeInfo(
            nickname=self.nickname,
            last_heartbeat=-1,
            node_name=contents.get("node_name", "???"),
            status=status,
            notes=note,
        )
        self._update_cb(info)


@sc.register_config("external_monitor", name="JsonMonitor", constructor=JsonMonitor)
@dataclass
class JsonMonitorConfig(sc.Config):
    pass
