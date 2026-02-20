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
from geometry_msgs.msg import WrenchStamped, TwistWithCovarianceStamped, Vector3Stamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from holoocean_interfaces.msg import AgentCommand, ControlCommand


class WrenchConverterNode(Node):
    """
    Converts HoloOcean agent/control command messages back to WrenchStamped messages.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("wrench_converter_node")

        self.declare_parameter("agent_topic", "/command/agent")
        self.declare_parameter("control_topic", "ControlCommand")
        self.declare_parameter("velocity_topic", "VelocitySensor")
        self.declare_parameter("wrench_raw_topic", "cmd_wrench_raw")
        self.declare_parameter("wrench_topic", "cmd_wrench")
        self.declare_parameter("wrench_frame", "com_link")
        self.declare_parameter("map_frame", "map")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        agent_topic = (
            self.get_parameter("agent_topic").get_parameter_value().string_value
        )
        control_topic = (
            self.get_parameter("control_topic").get_parameter_value().string_value
        )
        velocity_topic = (
            self.get_parameter("velocity_topic").get_parameter_value().string_value
        )
        wrench_raw_topic = (
            self.get_parameter("wrench_raw_topic").get_parameter_value().string_value
        )
        wrench_topic = (
            self.get_parameter("wrench_topic").get_parameter_value().string_value
        )
        self.wrench_frame = (
            self.get_parameter("wrench_frame").get_parameter_value().string_value
        )
        self.map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )

        self.agent_sub = self.create_subscription(
            AgentCommand, agent_topic, self.agent_callback, 10
        )
        self.control_sub = self.create_subscription(
            ControlCommand, control_topic, self.control_callback, 10
        )
        self.velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped, velocity_topic, self.velocity_callback, 10
        )
        self.wrench_raw_pub = self.create_publisher(WrenchStamped, wrench_raw_topic, 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, wrench_topic, 10)

        # From BlueROV2.h
        self.geo_factor = 0.70710678

        # From actuator.py
        self.rho = 1026.0
        self.d_prop = 0.14
        self.t_prop = 0.1
        self.kt_0 = 0.4566
        self.kt_max = 0.1798
        self.ja_max = 0.6632
        self.w = 0.056

        self.c1 = (1.0 - self.t_prop) * self.rho * pow(self.d_prop, 4) * self.kt_0
        self.c2 = (
            (1.0 - self.t_prop)
            * self.rho
            * pow(self.d_prop, 4)
            * (self.kt_max - self.kt_0)
            / self.ja_max
            * ((1 - self.w) / self.d_prop)
        )

        self.velocity = 0.0

        self.get_logger().info(
            f"Wrench converter started. Listening on {agent_topic} and {control_topic}, "
            f"publishing on {wrench_raw_topic} and {wrench_topic}."
        )

    def agent_callback(self, msg: AgentCommand):
        """
        Process agent command messages (BlueROV2).

        :param msg: AgentCommand message containing thruster values.
        """
        cmd = msg.command

        fwd = (cmd[4] + cmd[5] + cmd[6] + cmd[7]) * self.geo_factor
        lat = (cmd[4] - cmd[5] + cmd[6] - cmd[7]) * self.geo_factor
        vert = cmd[0] + cmd[1] + cmd[2] + cmd[3]

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = msg.header.stamp
        wrench_msg.header.frame_id = self.wrench_frame

        wrench_msg.wrench.force.x = fwd
        wrench_msg.wrench.force.y = lat
        wrench_msg.wrench.force.z = vert
        self.wrench_pub.publish(wrench_msg)

    def control_callback(self, msg: ControlCommand):
        """
        Process control command messages (CougUV).

        :param msg: ControlCommand message containing control surface/thruster values.
        """
        thruster_rpm = msg.cs[3]

        n_rps = thruster_rpm / 60.0

        # IMPORTANT! Assuming no spool up/down delays
        force_x_raw = self.c1 * abs(n_rps) * n_rps
        force_x = force_x_raw + self.c2 * n_rps * self.velocity

        raw_wrench_msg = WrenchStamped()
        raw_wrench_msg.header.stamp = msg.header.stamp
        raw_wrench_msg.header.frame_id = self.wrench_frame
        raw_wrench_msg.wrench.force.x = force_x_raw
        self.wrench_raw_pub.publish(raw_wrench_msg)

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = msg.header.stamp
        wrench_msg.header.frame_id = self.wrench_frame
        wrench_msg.wrench.force.x = force_x
        self.wrench_pub.publish(wrench_msg)

    def velocity_callback(self, msg: TwistWithCovarianceStamped):
        """
        Transform velocity into the body frame and store for control command processing.

        :param msg: Twist message containing current velocity in the world frame.
        """
        try:
            t_wrench_map = self.tf_buffer.lookup_transform(
                self.wrench_frame, self.map_frame, rclpy.time.Time()
            )

            # Transform velocity into the body frame
            vel_world = Vector3Stamped()
            vel_world.header = msg.header
            vel_world.vector = msg.twist.twist.linear
            vel_wrench = tf2_geometry_msgs.do_transform_vector3(vel_world, t_wrench_map)
            self.velocity = vel_wrench.vector.x

        except Exception:
            self.get_logger().error(
                "Could not transform DVL velocity.", throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    wrench_converter_node = WrenchConverterNode()
    try:
        rclpy.spin(wrench_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        wrench_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
