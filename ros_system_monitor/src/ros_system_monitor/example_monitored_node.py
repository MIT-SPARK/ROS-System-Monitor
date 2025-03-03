import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from ros_system_monitor_msgs.msg import NodeInfoMsg

from ros_system_monitor.node_info import NodeInfo, Status
import os

class ExampleMonitoredNode(Node):
    def __init__(self):
        super().__init__("example_monitored_node")

        nickname_d = ParameterDescriptor(
            type=Parameter.Type.STRING.value,
            description="Nodes to track",
        )
        self.declare_parameter("nickname", descriptor=nickname_d)
        self.nickname = self.get_parameter("nickname").value


        status_d = ParameterDescriptor(
            type=Parameter.Type.STRING.value,
            description="Nodes to track",
        )
        self.declare_parameter("status", descriptor=status_d)
        status_str = self.get_parameter("status").value

        if status_str == "NOMINAL":
            self.status = Status.NOMINAL
        elif status_str == "WARNING":
            self.status = Status.WARNING
        elif status_str == "ERROR":
            self.status = Status.ERROR
        elif status_str == "STARTUP":
            self.status = Status.STARTUP

        notes_d = ParameterDescriptor(
            type=Parameter.Type.STRING.value,
            description="Notes to publish",
        )
        self.declare_parameter("notes", descriptor=notes_d)
        self.notes = self.get_parameter("notes").value


        self.publisher = self.create_publisher(
            NodeInfoMsg, "~/node_status", 1
        )

        timer_period_s = 0.5
        self.timer = self.create_timer(timer_period_s, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        node_name = os.path.join(self.get_namespace(), self.get_name())
        info = NodeInfo(nickname=self.nickname, node_name=node_name, status=self.status.value, notes=self.notes)
        self.publisher.publish(info.to_ros())

def main(args=None):
    rclpy.init(args=args)
    node = ExampleMonitoredNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

