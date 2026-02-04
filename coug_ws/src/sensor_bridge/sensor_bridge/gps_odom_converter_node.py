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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class GpsOdomConverterNode(Node):
    """
    Transforms GPS Odometry data to the base_link frame.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("gps_odom_converter_node")

        self.declare_parameter("gps_odom_topic", "odometry/gps")
        self.declare_parameter("output_topic", "odometry/truth")
        self.declare_parameter("ahrs_topic", "imu/data")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_position_covariance", 0.01)

        gps_odom_topic = (
            self.get_parameter("gps_odom_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        ahrs_topic = self.get_parameter("ahrs_topic").get_parameter_value().string_value
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.max_position_covariance = (
            self.get_parameter("max_position_covariance")
            .get_parameter_value()
            .double_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.gps_subscription = self.create_subscription(
            Odometry, gps_odom_topic, self.listener_callback, 10
        )
        self.ahrs_subscription = self.create_subscription(
            Imu, ahrs_topic, self.ahrs_callback, 10
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        self.latest_ahrs_msg = None

        self.get_logger().info(
            f"GPS Odom converter started. Listening on {gps_odom_topic} and {ahrs_topic} "
            f"and publishing on {output_topic}."
        )

    def ahrs_callback(self, msg: Imu):
        """Cache latest AHRS message."""
        self.latest_ahrs_msg = msg

    def listener_callback(self, msg: Odometry):
        """
        Transform GPS Odometry data to the base_link frame.

        :param msg: Odometry message in the GPS frame.
        """
        try:
            t_gps_base = self.tf_buffer.lookup_transform(
                msg.child_frame_id, self.base_frame, rclpy.time.Time()
            )
        except Exception:
            self.get_logger().error(
                f"Could not find transform from {msg.child_frame_id} "
                f"to {self.base_frame}",
                throttle_duration_sec=1.0,
            )
            return

        if (
            msg.pose.covariance[0] > self.max_position_covariance
            or msg.pose.covariance[7] > self.max_position_covariance
        ):
            self.get_logger().warn(
                f"Dropping GPS message with high covariance: "
                f"{msg.pose.covariance[0]}, {msg.pose.covariance[7]}",
                throttle_duration_sec=1.0,
            )
            return

        p_base_in_map = do_transform_pose(msg.pose.pose, t_gps_base)

        if self.latest_ahrs_msg:
            try:
                t_ahrs_base = self.tf_buffer.lookup_transform(
                    self.latest_ahrs_msg.header.frame_id,
                    self.base_frame,
                    rclpy.time.Time(),
                )

                ahrs_pose = Pose()
                ahrs_pose.orientation = self.latest_ahrs_msg.orientation
                p_ahrs_base_in_map = do_transform_pose(ahrs_pose, t_ahrs_base)
                p_base_in_map.orientation = p_ahrs_base_in_map.orientation

            except Exception as e:
                self.get_logger().warn(
                    f"Could not transform AHRS orientation: {str(e)}",
                    throttle_duration_sec=1.0,
                )

        out_msg = Odometry()
        out_msg.header = msg.header
        out_msg.child_frame_id = self.base_frame
        out_msg.pose.pose = p_base_in_map
        out_msg.pose.covariance = msg.pose.covariance
        out_msg.twist = msg.twist

        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    gps_odom_converter_node = GpsOdomConverterNode()
    try:
        rclpy.spin(gps_odom_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_odom_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
