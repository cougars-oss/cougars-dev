#!/usr/bin/env python3
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

import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader
from matplotlib.patches import Circle


def get_odom_data(bag_path, target_topic="/odometry/global"):
    print(f"Reading {bag_path} for {target_topic}...")

    data = {
        "t": [],
        "pos": {"x": [], "y": [], "z": []},
        "rot": {"roll": [], "pitch": [], "yaw": []},
        "pos_sigma": {"x": [], "y": [], "z": []},
        "rot_sigma": {"roll": [], "pitch": [], "yaw": []},
    }

    start_t = None

    # Helper to convert quaternion to euler (simplified for plotting)
    def quat_to_euler(w, x, y, z):
        import math

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    with AnyReader([Path(bag_path)]) as reader:
        # Try to find the topic (handle namespaces)
        topic_connection = None
        for connection in reader.connections:
            if target_topic in connection.topic:
                topic_connection = connection
                break

        if not topic_connection:
            print(f"Error: Topic '{target_topic}' not found in bag.")
            # List available topics to help user
            print("Available topics:")
            for c in reader.connections:
                print(f"  {c.topic}")
            return None

        for connection, t, rawdata in reader.messages(connections=[topic_connection]):
            if start_t is None:
                start_t = t
            t_sec = (t - start_t) / 1e9
            msg = reader.deserialize(rawdata, connection.msgtype)

            data["t"].append(t_sec)

            # Position
            data["pos"]["x"].append(msg.pose.pose.position.x)
            data["pos"]["y"].append(msg.pose.pose.position.y)
            data["pos"]["z"].append(msg.pose.pose.position.z)

            # Rotation (Quaternion to Euler)
            q = msg.pose.pose.orientation
            r, p, y = quat_to_euler(q.w, q.x, q.y, q.z)
            data["rot"]["roll"].append(r)
            data["rot"]["pitch"].append(p)
            data["rot"]["yaw"].append(y)

            # Covariance (sigmas)
            # ROS Odometry pose.covariance is 6x6 row-major
            # 0:x, 7:y, 14:z, 21:roll, 28:pitch, 35:yaw
            data["pos_sigma"]["x"].append(np.sqrt(max(0, msg.pose.covariance[0])))
            data["pos_sigma"]["y"].append(np.sqrt(max(0, msg.pose.covariance[7])))
            data["pos_sigma"]["z"].append(np.sqrt(max(0, msg.pose.covariance[14])))
            data["rot_sigma"]["roll"].append(np.sqrt(max(0, msg.pose.covariance[21])))
            data["rot_sigma"]["pitch"].append(np.sqrt(max(0, msg.pose.covariance[28])))
            data["rot_sigma"]["yaw"].append(np.sqrt(max(0, msg.pose.covariance[35])))

    for key in data:
        if isinstance(data[key], dict):
            for sub in data[key]:
                data[key][sub] = np.array(data[key][sub])
        elif key == "t":
            data[key] = np.array(data[key])

    return data


def plot_covariances(bag_path):
    data = get_odom_data(bag_path)
    if data is None:
        return

    t = data["t"]

    # --- FIGURE 1: Sigmas over Time ---
    fig1, axes1 = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Position Sigmas
    axes1[0].plot(t, data["pos_sigma"]["x"], label=r"$\sigma_x$")
    axes1[0].plot(t, data["pos_sigma"]["y"], label=r"$\sigma_y$")
    axes1[0].plot(t, data["pos_sigma"]["z"], label=r"$\sigma_z$")
    axes1[0].set_ylabel("Position Sigma (m)")
    axes1[0].set_title("Pose Uncertainty Over Time")
    axes1[0].legend()
    axes1[0].grid(True, alpha=0.3)

    # Rotation Sigmas
    axes1[1].plot(t, np.degrees(data["rot_sigma"]["roll"]), label=r"$\sigma_{roll}$")
    axes1[1].plot(t, np.degrees(data["rot_sigma"]["pitch"]), label=r"$\sigma_{pitch}$")
    axes1[1].plot(t, np.degrees(data["rot_sigma"]["yaw"]), label=r"$\sigma_{yaw}$")
    axes1[1].set_ylabel("Rotation Sigma (deg)")
    axes1[1].set_xlabel("Time (s)")
    axes1[1].legend()
    axes1[1].grid(True, alpha=0.3)

    plt.tight_layout()

    # --- FIGURE 2: XY Trajectory with Circle Covariances ---
    plt.figure(figsize=(8, 8))
    x = data["pos"]["x"]
    y = data["pos"]["y"]
    sigma_x = data["pos_sigma"]["x"]
    sigma_y = data["pos_sigma"]["y"]

    plt.plot(x, y, "b-", label="Trajectory", alpha=0.6)

    # Plot circles at intervals
    # Horizontal uncertainty sigma_h = sqrt(sigma_x^2 + sigma_y^2)
    step = max(1, len(x) // 500)  # Draw ~500 circles
    for i in range(0, len(x), step):
        # We'll plot a circle with radius 2*sigma (approx 95% confidence for circular)
        # Or just 1*sigma if preferred. Let's do 2*sigma for visibility.
        radius = 2 * np.sqrt(sigma_x[i] ** 2 + sigma_y[i] ** 2)
        circle = Circle((x[i], y[i]), radius, color="r", fill=False, alpha=1.0)
        plt.gca().add_patch(circle)

    # Plot last point circle
    radius_last = 2 * np.sqrt(sigma_x[-1] ** 2 + sigma_y[-1] ** 2)
    circle_last = Circle(
        (x[-1], y[-1]),
        radius_last,
        color="r",
        fill=True,
        alpha=1.0,
        label=r"2$\sigma$ Uncertainty",
    )
    plt.gca().add_patch(circle_last)

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("XY Trajectory with 2-Sigma Uncertainty Circles")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: plot_covariances.py <bag_file>")
        sys.exit(1)

    bag_path = sys.argv[1]
    plot_covariances(bag_path)
