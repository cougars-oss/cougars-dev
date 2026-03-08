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
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped


class ImuConverterNode(Node):
    """
    Converts IMU data from HoloOcean to standard IMU messages.

    Injects Gaussian noise and simulated random-walk bias to replicate
    realistic IMU sensor behavior.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("imu_converter_node")

        self.declare_parameter("input_topic", "IMUSensor")
        self.declare_parameter("output_topic", "imu/data")
        self.declare_parameter("bias_topic", "imu_bias")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("accel_noise_sigmas", [0.0078, 0.0078, 0.0078])
        self.declare_parameter("gyro_noise_sigmas", [0.0012, 0.0012, 0.0012])
        self.declare_parameter("add_noise", True)
        self.declare_parameter("add_bias", True)
        self.declare_parameter("accel_bias_rw_sigmas", [1.05e-5, 1.05e-5, 1.05e-5])
        self.declare_parameter("gyro_bias_rw_sigmas", [3.91e-6, 3.91e-6, 3.91e-6])

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        bias_topic = self.get_parameter("bias_topic").get_parameter_value().string_value
        self.imu_frame = (
            self.get_parameter("imu_frame").get_parameter_value().string_value
        )
        self.accel_noise_sigmas = (
            self.get_parameter("accel_noise_sigmas")
            .get_parameter_value()
            .double_array_value
        )
        self.gyro_noise_sigmas = (
            self.get_parameter("gyro_noise_sigmas")
            .get_parameter_value()
            .double_array_value
        )
        self.add_noise = (
            self.get_parameter("add_noise").get_parameter_value().bool_value
        )
        self.add_bias = self.get_parameter("add_bias").get_parameter_value().bool_value
        self.accel_bias_rw_sigmas = (
            self.get_parameter("accel_bias_rw_sigmas")
            .get_parameter_value()
            .double_array_value
        )
        self.gyro_bias_rw_sigmas = (
            self.get_parameter("gyro_bias_rw_sigmas")
            .get_parameter_value()
            .double_array_value
        )

        self.accel_bias = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.last_stamp = None

        self.subscription = self.create_subscription(
            Imu, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Imu, output_topic, 10)
        self.bias_publisher = self.create_publisher(
            TwistWithCovarianceStamped, bias_topic, 10
        )

        self.get_logger().info(
            f"IMU converter started. Listening on {input_topic} and publishing on {output_topic} "
            + f"and {bias_topic}."
        )

    def listener_callback(self, msg: Imu):
        """
        Process IMU sensor data (Imu).

        :param msg: Imu message containing IMU data.
        """
        msg.header.frame_id = self.imu_frame

        if self.add_bias:
            current_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.last_stamp is not None:
                dt = current_stamp - self.last_stamp
                if dt > 0.0:
                    sqrt_dt = math.sqrt(dt)
                    for i in range(3):
                        self.accel_bias[i] += random.gauss(
                            0, self.accel_bias_rw_sigmas[i] * sqrt_dt
                        )
                        self.gyro_bias[i] += random.gauss(
                            0, self.gyro_bias_rw_sigmas[i] * sqrt_dt
                        )
            self.last_stamp = current_stamp

            msg.linear_acceleration.x += self.accel_bias[0]
            msg.linear_acceleration.y += self.accel_bias[1]
            msg.linear_acceleration.z += self.accel_bias[2]

            msg.angular_velocity.x += self.gyro_bias[0]
            msg.angular_velocity.y += self.gyro_bias[1]
            msg.angular_velocity.z += self.gyro_bias[2]

        if self.add_noise:
            msg.linear_acceleration.x += random.gauss(0, self.accel_noise_sigmas[0])
            msg.linear_acceleration.y += random.gauss(0, self.accel_noise_sigmas[1])
            msg.linear_acceleration.z += random.gauss(0, self.accel_noise_sigmas[2])

            msg.angular_velocity.x += random.gauss(0, self.gyro_noise_sigmas[0])
            msg.angular_velocity.y += random.gauss(0, self.gyro_noise_sigmas[1])
            msg.angular_velocity.z += random.gauss(0, self.gyro_noise_sigmas[2])

        msg.linear_acceleration_covariance[0] = self.accel_noise_sigmas[0] ** 2
        msg.linear_acceleration_covariance[4] = self.accel_noise_sigmas[1] ** 2
        msg.linear_acceleration_covariance[8] = self.accel_noise_sigmas[2] ** 2

        msg.angular_velocity_covariance[0] = self.gyro_noise_sigmas[0] ** 2
        msg.angular_velocity_covariance[4] = self.gyro_noise_sigmas[1] ** 2
        msg.angular_velocity_covariance[8] = self.gyro_noise_sigmas[2] ** 2

        self.publisher.publish(msg)

        bias_msg = TwistWithCovarianceStamped()
        bias_msg.header = msg.header
        bias_msg.twist.twist.linear.x = self.accel_bias[0]
        bias_msg.twist.twist.linear.y = self.accel_bias[1]
        bias_msg.twist.twist.linear.z = self.accel_bias[2]
        bias_msg.twist.twist.angular.x = self.gyro_bias[0]
        bias_msg.twist.twist.angular.y = self.gyro_bias[1]
        bias_msg.twist.twist.angular.z = self.gyro_bias[2]

        self.bias_publisher.publish(bias_msg)


def main(args=None):
    rclpy.init(args=args)
    imu_converter_node = ImuConverterNode()
    try:
        rclpy.spin(imu_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
