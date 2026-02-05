#!/usr/bin/env python3

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

import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPathNode(Node):
    """
    Dynamically converts odometry topics to path topics for visualization.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("odom_to_path_node")

        self.declare_parameter("check_topic_rate", 2.0)
        self.check_rate = (
            self.get_parameter("check_topic_rate").get_parameter_value().double_value
        )

        self.subs = {}
        self.pubs = {}
        self.paths = {}

        self.timer = self.create_timer(self.check_rate, self.check_topics)
        self.get_logger().info(
            "OdomToPathNode started. Searching for 'odometry' topics..."
        )

    def check_topics(self):
        """Check for new odometry topics and create converters for them."""
        topic_names_and_types = self.get_topic_names_and_types()

        for topic_name, topic_types in topic_names_and_types:
            if "nav_msgs/msg/Odometry" in topic_types and "odometry" in topic_name:
                if topic_name not in self.subs:
                    self.create_converter(topic_name)

    def create_converter(self, topic_name: str):
        """
        Create a subscriber/publisher pair for a specific odometry topic.

        :param topic_name: The name of the odometry topic.
        """
        new_topic_name = topic_name.replace("odometry", "path")

        self.get_logger().info(f"Converting {topic_name} -> {new_topic_name}")

        self.pubs[topic_name] = self.create_publisher(Path, new_topic_name, 10)
        self.paths[topic_name] = Path()

        self.subs[topic_name] = self.create_subscription(
            Odometry,
            topic_name,
            lambda msg, source=topic_name: self.odom_callback(msg, source),
            10,
        )

    def odom_callback(self, msg: Odometry, source_topic: str):
        """
        Process odometry data and append to path.

        :param msg: The incoming odometry message.
        :param source_topic: The topic name it came from.
        """
        path_msg = self.paths[source_topic]
        path_msg.header = msg.header

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        path_msg.poses.append(pose_stamped)

        self.pubs[source_topic].publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPathNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
