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

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped


def get_quaternion_from_euler(roll, pitch, yaw):
    """Convert an Euler angle to a quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy
    q_w = cr * cp * cy + sr * sp * sy

    return [q_x, q_y, q_z, q_w]


class AhrsConverterNode(Node):
    """
    Converts AHRS data from HoloOcean to standard IMU messages and adds noise.

    :author: Nelson Durrant
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("ahrs_converter_node")

        self.declare_parameter("input_topic", "RotationSensor")
        self.declare_parameter("output_topic", "imu/ahrs")
        self.declare_parameter("ahrs_frame", "imu_link")
        self.declare_parameter("noise_sigmas", [0.00349, 0.00349, 0.01745])
        self.declare_parameter("add_noise", True)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.ahrs_frame = (
            self.get_parameter("ahrs_frame").get_parameter_value().string_value
        )
        self.noise_sigmas = (
            self.get_parameter("noise_sigmas").get_parameter_value().double_array_value
        )
        self.add_noise = (
            self.get_parameter("add_noise").get_parameter_value().bool_value
        )

        self.subscription = self.create_subscription(
            Vector3Stamped,
            input_topic,
            self.listener_callback,
            qos_profile_system_default,
        )
        self.publisher = self.create_publisher(
            Imu, output_topic, qos_profile_system_default
        )

        self.get_logger().info(
            f"AHRS converter started. Listening on {input_topic} and "
            f"publishing on {output_topic}."
        )

    def listener_callback(self, msg: Vector3Stamped):
        """
        Process rotation sensor data (Vector3Stamped).

        :param msg: Vector3Stamped message containing Euler angles.
        """
        imu_msg = Imu()
        imu_msg.header.stamp = msg.header.stamp
        imu_msg.header.frame_id = self.ahrs_frame

        roll_deg = msg.vector.x
        pitch_deg = msg.vector.y
        yaw_deg = msg.vector.z

        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        if self.add_noise:
            roll_rad += random.gauss(0, self.noise_sigmas[0])
            pitch_rad += random.gauss(0, self.noise_sigmas[1])
            yaw_rad += random.gauss(0, self.noise_sigmas[2])

        q_array = get_quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        imu_msg.orientation.x = q_array[0]
        imu_msg.orientation.y = q_array[1]
        imu_msg.orientation.z = q_array[2]
        imu_msg.orientation.w = q_array[3]

        imu_msg.orientation_covariance[0] = self.noise_sigmas[0] ** 2
        imu_msg.orientation_covariance[4] = self.noise_sigmas[1] ** 2
        imu_msg.orientation_covariance[8] = self.noise_sigmas[2] ** 2

        self.publisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    ahrs_converter_node = AhrsConverterNode()
    try:
        rclpy.spin(ahrs_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        ahrs_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
