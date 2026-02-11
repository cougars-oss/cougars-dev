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
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from dvl_msgs.msg import DVLDR
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose


class DvlOdomConverterNode(Node):
    """
    Converts DVL dead-reckoned data to odometry messages.

    Optionally publishes the dvl_odom->base_link transform.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("dvl_odom_converter_node")

        self.declare_parameter("input_topic", "dvl/position")
        self.declare_parameter("output_topic", "odometry/dvl")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("dvl_odom_frame", "dvl_odom")
        self.declare_parameter("publish_local_tf", False)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.dvl_frame = (
            self.get_parameter("dvl_frame").get_parameter_value().string_value
        )
        self.dvl_odom_frame = (
            self.get_parameter("dvl_odom_frame").get_parameter_value().string_value
        )
        self.publish_local_tf = (
            self.get_parameter("publish_local_tf").get_parameter_value().bool_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            DVLDR, input_topic, self.listener_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info(
            f"DVL odom converter started. Listening on {input_topic} "
            f"and publishing on {output_topic}."
        )

    def listener_callback(self, msg: DVLDR):
        """
        Process dead-reckoned position data (DVLDR message).

        :param msg: DVLDR message containing dead reckoning data.
        """
        try:
            t_dvl_base = self.tf_buffer.lookup_transform(
                self.dvl_frame, self.base_frame, rclpy.time.Time()
            )

            p_base_in_dvl = PoseStamped()
            p_base_in_dvl.header.frame_id = self.dvl_frame
            p_base_in_dvl.pose.position.x = t_dvl_base.transform.translation.x
            p_base_in_dvl.pose.position.y = t_dvl_base.transform.translation.y
            p_base_in_dvl.pose.position.z = t_dvl_base.transform.translation.z
            p_base_in_dvl.pose.orientation = t_dvl_base.transform.rotation

            t_odom_dvl = TransformStamped()
            t_odom_dvl.header.frame_id = self.dvl_odom_frame
            t_odom_dvl.child_frame_id = self.dvl_frame
            t_odom_dvl.transform.translation.x = msg.position.x
            t_odom_dvl.transform.translation.y = msg.position.y
            t_odom_dvl.transform.translation.z = msg.position.z
            t_odom_dvl.transform.rotation = self.quaternion_from_euler(
                math.radians(msg.roll), math.radians(msg.pitch), math.radians(msg.yaw)
            )

            p_base_in_odom = do_transform_pose(p_base_in_dvl.pose, t_odom_dvl)

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()

            sec = int(msg.time)
            nanosec = int((msg.time - sec) * 1e9)
            odom.header.stamp.sec = sec
            odom.header.stamp.nanosec = nanosec

            odom.header.frame_id = self.dvl_odom_frame
            odom.child_frame_id = self.base_frame

            odom.pose.pose = p_base_in_odom

            var = msg.pos_std**2
            odom.pose.covariance[0] = var
            odom.pose.covariance[7] = var
            odom.pose.covariance[14] = var

            self.publisher.publish(odom)

            if self.publish_local_tf:
                ts = TransformStamped()
                ts.header = odom.header
                ts.child_frame_id = odom.child_frame_id
                ts.transform.translation.x = p_base_in_odom.position.x
                ts.transform.translation.y = p_base_in_odom.position.y
                ts.transform.translation.z = p_base_in_odom.position.z
                ts.transform.rotation = p_base_in_odom.orientation
                self.tf_broadcaster.sendTransform(ts)

        except Exception as e:
            self.get_logger().error(f"Transform error: {e}", throttle_duration_sec=1.0)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles to a Quaternion message.

        :param roll: Roll angle in radians.
        :param pitch: Pitch angle in radians.
        :param yaw: Yaw angle in radians.
        :return: Populated geometry_msgs/Quaternion message.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main(args=None):
    rclpy.init(args=args)
    dvl_odom_converter_node = DvlOdomConverterNode()
    try:
        rclpy.spin(dvl_odom_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_odom_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
