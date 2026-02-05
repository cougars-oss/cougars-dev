# Copyright (c) 2026 BYU FRoSt Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from holoocean_interfaces.msg import DesiredCommand
from std_msgs.msg import Header


class HsdConverterNode(Node):
    """
    Converts standard ROS 2 command messages into HoloOcean desired command messages.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("hsd_converter_node")

        self.declare_parameter("heading_topic", "heading")
        self.declare_parameter("speed_topic", "speed")
        self.declare_parameter("depth_topic", "depth")
        self.declare_parameter("output_heading_topic", "/heading")
        self.declare_parameter("output_speed_topic", "/speed")
        self.declare_parameter("output_depth_topic", "/depth")
        self.declare_parameter("agent_name", "auv0")

        self.input_heading_topic = (
            self.get_parameter("heading_topic").get_parameter_value().string_value
        )
        self.input_speed_topic = (
            self.get_parameter("speed_topic").get_parameter_value().string_value
        )
        self.input_depth_topic = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        self.output_heading_topic = (
            self.get_parameter("output_heading_topic")
            .get_parameter_value()
            .string_value
        )
        self.output_speed_topic = (
            self.get_parameter("output_speed_topic").get_parameter_value().string_value
        )
        self.output_depth_topic = (
            self.get_parameter("output_depth_topic").get_parameter_value().string_value
        )
        self.agent_name = (
            self.get_parameter("agent_name").get_parameter_value().string_value
        )

        self.output_heading_pub = self.create_publisher(
            DesiredCommand, self.output_heading_topic, 10
        )
        self.output_speed_pub = self.create_publisher(
            DesiredCommand, self.output_speed_topic, 10
        )
        self.output_depth_pub = self.create_publisher(
            DesiredCommand, self.output_depth_topic, 10
        )

        self.heading_sub = self.create_subscription(
            Float64, self.input_heading_topic, self.heading_callback, 10
        )
        self.speed_sub = self.create_subscription(
            Float64, self.input_speed_topic, self.speed_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Float64, self.input_depth_topic, self.depth_callback, 10
        )

        self.get_logger().info(
            f"HSD converter started. Listening on {self.input_heading_topic}, "
            f"{self.input_speed_topic}, and {self.input_depth_topic} and publishing "
            f"on {self.output_heading_topic}, {self.output_speed_topic}, and "
            f"{self.output_depth_topic}."
        )

    def create_command_msg(self, value):
        """
        Create a DesiredCommand message.

        :param value: The value (heading, speed, or depth) to put in the message.
        :return: Populated DesiredCommand message.
        """
        msg = DesiredCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.agent_name
        msg.data = float(value)
        return msg

    def heading_callback(self, msg: Float64):
        """
        Process heading messages (Float64).

        :param msg: Float64 message containing the desired ENU heading.
        """
        out_msg = self.create_command_msg(msg.data)
        self.output_heading_pub.publish(out_msg)

    def speed_callback(self, msg: Float64):
        """
        Process speed messages (Float64).

        :param msg: Float64 message containing the desired speed.
        """
        out_msg = self.create_command_msg(msg.data)
        self.output_speed_pub.publish(out_msg)

    def depth_callback(self, msg: Float64):
        """
        Process depth messages (Float64).

        :param msg: Float64 message containing the desired depth.
        """
        out_msg = self.create_command_msg(msg.data)
        self.output_depth_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HsdConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
