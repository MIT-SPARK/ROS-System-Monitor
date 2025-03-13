import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from ros_system_monitor_msgs.msg import NodeInfoMsg

import ros_system_monitor as rsm


def _str_descriptor(description: str) -> ParameterDescriptor:
    return ParameterDescriptor(
        type=Parameter.Type.STRING.value, description=description
    )


class ExampleMonitoredNode(Node):
    """Example node that publishes status parsed from parameters."""

    def __init__(self):
        super().__init__("example_monitored_node")

        self.declare_parameter("nickname", "", _str_descriptor("Tracked Node"))
        self.nickname = self.get_parameter("nickname").value
        assert self.nickname != "", "nickname param required."

        self.declare_parameter("status", "", _str_descriptor("Node status"))
        status_str = self.get_parameter("status").value
        assert status_str != "", "status param required."

        self.status, status_note = rsm.str_to_status(status_str)

        self.declare_parameter("notes", "", _str_descriptor("Node notes"))
        self.notes = self.get_parameter("notes").value
        if status_note is not None:
            self.notes += f" ({status_note})"

        self.publisher = self.create_publisher(NodeInfoMsg, "~/node_status", 1)

        timer_period_s = 0.5
        self.timer = self.create_timer(timer_period_s, self.timer_callback)

    def timer_callback(self):
        msg = NodeInfoMsg()
        msg.nickname = self.nickname
        msg.node_name = self.get_fully_qualified_name()
        msg.status = self.status.value
        msg.notes = self.notes
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExampleMonitoredNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
