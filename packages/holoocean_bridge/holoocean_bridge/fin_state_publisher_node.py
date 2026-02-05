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
from holoocean_interfaces.msg import ControlCommand
from sensor_msgs.msg import JointState


class FinStatePublisherNode(Node):
    """
    Publishes fin states based on control command messages from HoloOcean.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("fin_state_publisher_node")

        self.declare_parameter("input_topic", "ControlCommand")
        self.declare_parameter("output_topic", "joint_states")

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        self.joint_names = ["top_fin_joint", "left_fin_joint", "right_fin_joint"]

        self.subscription = self.create_subscription(
            ControlCommand, input_topic, self.control_command_callback, 10
        )
        self.publisher = self.create_publisher(JointState, output_topic, 10)

        self.get_logger().info(
            f"Fin state publisher started. Listening on {input_topic} "
            f"and publishing on {output_topic}."
        )

    def control_command_callback(self, msg: ControlCommand):
        """
        Process HoloOcean control commands (ControlCommand).

        :param msg: ControlCommand message containing fin states.
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names

        # HoloOcean uses the order: [rudder, starboard_elevator, port_elevator, thruster]
        # We map this to the URDF joints: [top_fin_joint, left_fin_joint, right_fin_joint]
        rudder = -msg.cs[0]
        starboard_elevator = msg.cs[1]
        port_elevator = msg.cs[2]
        joint_state.position = [rudder, port_elevator, starboard_elevator]
        self.publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    fin_state_publisher_node = FinStatePublisherNode()
    try:
        rclpy.spin(fin_state_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        fin_state_publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
