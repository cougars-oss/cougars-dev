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

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("cmd_vel_converter_node")

        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "/command/agent")
        self.declare_parameter("agent_name", "auv0")

        # Calculated from BlueROV2.h
        self.declare_parameter("thruster_limit", 28.75)
        self.declare_parameter("horizontal_scale", 4.066)
        self.declare_parameter("vertical_scale", 2.875)
        self.declare_parameter("angular_scale", 2.0)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.agent_name = (
            self.get_parameter("agent_name").get_parameter_value().string_value
        )
        self.thruster_limit = (
            self.get_parameter("thruster_limit").get_parameter_value().double_value
        )
        self.h_scale = (
            self.get_parameter("horizontal_scale").get_parameter_value().double_value
        )
        self.v_scale = (
            self.get_parameter("vertical_scale").get_parameter_value().double_value
        )
        self.a_scale = (
            self.get_parameter("angular_scale").get_parameter_value().double_value
        )

        self.subscription = self.create_subscription(
            Twist, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(AgentCommand, output_topic, 10)

        self.get_logger().info(
            f"Cmd Vel converter started. Listening on {input_topic} and "
            f"publishing on {output_topic}."
        )

    def listener_callback(self, msg: Twist):
        """
        Process cmd_vel (Twist) messages.

        :param msg: Twist message containing linear and angular velocities.
        """
        agent_cmd = AgentCommand()
        agent_cmd.header.stamp = self.get_clock().now().to_msg()
        agent_cmd.header.frame_id = self.agent_name

        # Map Twist to BlueROV thruster commands
        fwd = msg.linear.x * self.h_scale
        lat = msg.linear.y * self.h_scale
        vert = msg.linear.z * self.v_scale

        roll = msg.angular.x * self.a_scale
        pitch = msg.angular.y * self.a_scale
        yaw = msg.angular.z * self.a_scale

        cmd_0 = vert - pitch - roll
        cmd_1 = vert - pitch + roll
        cmd_2 = vert + pitch + roll
        cmd_3 = vert + pitch - roll

        cmd_4 = fwd + lat + yaw
        cmd_5 = fwd - lat - yaw
        cmd_6 = fwd + lat - yaw
        cmd_7 = fwd - lat + yaw

        raw_cmds = [cmd_0, cmd_1, cmd_2, cmd_3, cmd_4, cmd_5, cmd_6, cmd_7]

        max_req = max([abs(x) for x in raw_cmds])
        if max_req > self.thruster_limit:
            scale_factor = self.thruster_limit / max_req
            final_cmds = [x * scale_factor for x in raw_cmds]
        else:
            final_cmds = raw_cmds

        agent_cmd.command = final_cmds
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
