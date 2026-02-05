# Copyright (c) 2026 BYU FROST Lab
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
from geometry_msgs.msg import Twist
from holoocean_interfaces.msg import AgentCommand


class CmdVelConverterNode(Node):
    """
    Converts ROS 2 cmd_vel commands to HoloOcean agent command messages.

    TODO: Adjust scalars so the velocities actually kind of match.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("cmd_vel_converter_node")

        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "/command/agent")
        self.declare_parameter("agent_name", "auv0")

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.agent_name = (
            self.get_parameter("agent_name").get_parameter_value().string_value
        )

        self.last_command_msg = None

        self.subscription = self.create_subscription(
            Twist, input_topic, self.cmd_vel_callback, 10
        )
        self.publisher = self.create_publisher(AgentCommand, output_topic, 10)
        self.get_logger().info(
            f"Cmd Vel converter started. Listening on {input_topic} and "
            f"publishing on {output_topic}."
        )

    def cmd_vel_callback(self, msg: Twist):
        """
        Process cmd_vel (Twist) messages.

        :param msg: Twist message containing linear and angular velocities.
        """
        agent_cmd = AgentCommand()
        agent_cmd.header.stamp = self.get_clock().now().to_msg()
        agent_cmd.header.frame_id = self.agent_name

        # Map Twist 6-DOF to BlueROV variables
        forward = msg.linear.x * 10
        vertical = msg.linear.z * 10
        yaw = msg.angular.z * 0.1
        left = msg.linear.y * 10
        pitch = msg.angular.y * 0.1
        roll = msg.angular.x * 0.1

        agent_cmd.command = [
            (vertical + pitch + roll),
            (vertical + pitch - roll),
            (vertical - pitch - roll),
            (vertical - pitch + roll),
            (forward + yaw + left),
            (forward - yaw - left),
            (forward - yaw + left),
            (forward + yaw - left),
        ]

        self.publisher.publish(agent_cmd)


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_converter = CmdVelConverterNode()
    try:
        rclpy.spin(cmd_vel_converter)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_converter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
