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
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import message_filters
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose


class TruthConverterNode(Node):
    """
    Converts ground truth data from HoloOcean to an odometry message.

    Optionally publishes the map->base_link transform.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("truth_converter_node")

        self.declare_parameter("input_topic", "auv0/LocationSensor")
        self.declare_parameter("imu_topic", "auv0/DynamicsSensorIMU")
        self.declare_parameter("output_topic", "odometry/truth")
        self.declare_parameter("com_frame_id", "com_link")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_tf", False)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.com_frame_id = (
            self.get_parameter("com_frame_id").get_parameter_value().string_value
        )
        self.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Set up message filters for synchronization
        location_sub = message_filters.Subscriber(
            self, PoseWithCovarianceStamped, input_topic
        )
        imu_sub = message_filters.Subscriber(self, Imu, imu_topic)

        ts = message_filters.TimeSynchronizer([location_sub, imu_sub], 10)
        ts.registerCallback(self.sync_callback)

        self.get_logger().info(
            f"Truth converter started. Listening on {input_topic} and {imu_topic} "
            f"and publishing on {output_topic}."
        )

    def sync_callback(self, location_msg: PoseWithCovarianceStamped, imu_msg: Imu):
        """
        Process synchronized location (PoseWithCovarianceStamped) and IMU (Imu) messages.

        :param location_msg: PoseWithCovarianceStamped message containing location data.
        :param imu_msg: Imu message containing orientation data.
        """
        try:
            t_com_base = self.tf_buffer.lookup_transform(
                self.com_frame_id, self.child_frame_id, rclpy.time.Time()
            )
        except Exception:
            self.get_logger().error(
                f"Could not find transform from {self.com_frame_id} "
                f"to {self.child_frame_id}",
                throttle_duration_sec=1.0
            )
            return

        p_com_in_holo = PoseStamped()
        p_com_in_holo.header = location_msg.header
        p_com_in_holo.header.frame_id = "holoocean"
        p_com_in_holo.pose.position = location_msg.pose.pose.position
        p_com_in_holo.pose.orientation = imu_msg.orientation

        try:
            p_com_in_map = self.tf_buffer.transform(
                p_com_in_holo, "map", timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception:
            self.get_logger().error(
                "Could not transform pose from holoocean to map",
                throttle_duration_sec=1.0,
            )
            return

        t_map_com = TransformStamped()
        t_map_com.header = location_msg.header
        t_map_com.child_frame_id = self.com_frame_id
        t_map_com.transform.translation.x = p_com_in_map.pose.position.x
        t_map_com.transform.translation.y = p_com_in_map.pose.position.y
        t_map_com.transform.translation.z = p_com_in_map.pose.position.z
        t_map_com.transform.rotation = p_com_in_map.pose.orientation

        p_base_in_com = PoseStamped()
        p_base_in_com.pose.position.x = t_com_base.transform.translation.x
        p_base_in_com.pose.position.y = t_com_base.transform.translation.y
        p_base_in_com.pose.position.z = t_com_base.transform.translation.z
        p_base_in_com.pose.orientation = t_com_base.transform.rotation

        p_base_in_map = do_transform_pose(p_base_in_com.pose, t_map_com)

        odom_msg = Odometry()
        odom_msg.header = location_msg.header
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose = p_base_in_map
        self.publisher.publish(odom_msg)

        if self.get_parameter("publish_tf").get_parameter_value().bool_value:
            t = TransformStamped()
            t.header.stamp = location_msg.header.stamp
            t.header.frame_id = "map"
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = p_base_in_map.position.x
            t.transform.translation.y = p_base_in_map.position.y
            t.transform.translation.z = p_base_in_map.position.z
            t.transform.rotation = p_base_in_map.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    truth_converter_node = TruthConverterNode()
    try:
        rclpy.spin(truth_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        truth_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
