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

import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from dvl_msgs.msg import DVL


class DvlConverterNode(Node):
    """
    Converts DVL data from HoloOcean to Waterlinked DVL messages.

    Injects beam-based Gaussian noise to replicate HoloOcean's internal sensor
    noise model (Janus configuration).

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("dvl_converter_node")

        self.declare_parameter("input_topic", "DVLSensorVelocity")
        self.declare_parameter("output_topic", "dvl/data")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("noise_sigmas", [0.02, 0.02, 0.02])
        self.declare_parameter("add_noise", True)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.dvl_frame = (
            self.get_parameter("dvl_frame").get_parameter_value().string_value
        )
        self.noise_sigmas = (
            self.get_parameter("noise_sigmas").get_parameter_value().double_array_value
        )
        self.add_noise = (
            self.get_parameter("add_noise").get_parameter_value().bool_value
        )

        self.subscription = self.create_subscription(
            TwistWithCovarianceStamped, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(DVL, output_topic, 10)

        self.get_logger().info(
            f"DVL converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: TwistWithCovarianceStamped):
        """
        Process DVL sensor data (TwistWithCovarianceStamped).

        :param msg: TwistWithCovarianceStamped message containing DVL data.
        """
        msg.header.frame_id = self.dvl_frame

        dvl_msg = DVL()
        dvl_msg.header = msg.header
        dvl_msg.header.frame_id = self.dvl_frame

        if self.add_noise:
            noise_x = random.gauss(0, self.noise_sigmas[0])
            noise_y = random.gauss(0, self.noise_sigmas[1])
            noise_z = random.gauss(0, self.noise_sigmas[2])
        else:
            noise_x = 0.0
            noise_y = 0.0
            noise_z = 0.0

        dvl_msg.velocity.x = msg.twist.twist.linear.x + noise_x
        dvl_msg.velocity.y = msg.twist.twist.linear.y + noise_y
        dvl_msg.velocity.z = msg.twist.twist.linear.z + noise_z

        dvl_msg.velocity_valid = True

        # Convert nanoseconds to microseconds
        dvl_msg.time_of_validity = int(
            msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3
        )

        dvl_msg.covariance = [0.0] * 9
        dvl_msg.covariance[0] = self.noise_sigmas[0] ** 2
        dvl_msg.covariance[4] = self.noise_sigmas[1] ** 2
        dvl_msg.covariance[8] = self.noise_sigmas[2] ** 2

        self.publisher.publish(dvl_msg)


def main(args=None):
    rclpy.init(args=args)
    dvl_converter_node = DvlConverterNode()
    try:
        rclpy.spin(dvl_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
