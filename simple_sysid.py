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

# Weighted LLS
# Not relevant for sim data, covs are constant
ADD_COV_WEIGHTS = True

# Gaussian Priors (from class)
# The results ended up converging well w/o these
PRIOR_MEANS = {
    "x": np.array([0.0, 0.0, 0.0]),
    "y": np.array([0.0, 0.0, 0.0]),
    "z": np.array([0.0, 0.0, 0.0]),
}
PRIOR_SIGMAS = {
    "x": np.array([1e3, 1e3, 1e3]),
    "y": np.array([1e3, 1e3, 1e3]),
    "z": np.array([1e3, 1e3, 1e3]),
}

# Simplified HoloOcean URDFs
SENSOR_INVERSIONS = {
    "coug0sim": {"imu": True, "dvl": True},
    "blue0sim": {"imu": True, "dvl": False},
}


def get_namespace(bag_path):
    name = Path(bag_path).name.lower()
    if "blue" in name:
        return "blue0sim"
    elif "coug" in name:
        return "coug0sim"


def get_data(bag_path, auv_ns):
    print(f"Reading {bag_path} for {auv_ns}...")

    topic_force = "/" + auv_ns + "/cmd_wrench"
    topic_accel = "/" + auv_ns + "/imu/data_raw"
    topic_vel = "/" + auv_ns + "/dvl/twist"
    topic_orientation = "/" + auv_ns + "/imu/data"

    data = {
        "F": {"x": [], "y": [], "z": []},
        "A": {"x": [], "y": [], "z": []},
        "V": {"x": [], "y": [], "z": []},
        "A_cov": {"x": [], "y": [], "z": []},
        "V_cov": {"x": [], "y": [], "z": []},
        "Q": [],
    }

    start_t = None
    with AnyReader([Path(bag_path)]) as reader:
        connections = [
            x
            for x in reader.connections
            if x.topic in [topic_force, topic_accel, topic_vel, topic_orientation]
        ]

        for connection, t, rawdata in reader.messages(connections=connections):
            if start_t is None:
                start_t = t
            t_sec = (t - start_t) / 1e9
            msg = reader.deserialize(rawdata, connection.msgtype)

            if connection.topic == topic_force:
                data["F"]["x"].append([t_sec, msg.wrench.force.x])
                data["F"]["y"].append([t_sec, msg.wrench.force.y])
                data["F"]["z"].append([t_sec, msg.wrench.force.z])

            elif connection.topic == topic_accel:
                # Account for IMU rotations in HoloOcean
                flip = -1 if SENSOR_INVERSIONS[auv_ns]["imu"] else 1
                data["A"]["x"].append([t_sec, msg.linear_acceleration.x])
                data["A"]["y"].append([t_sec, msg.linear_acceleration.y * flip])
                data["A"]["z"].append([t_sec, msg.linear_acceleration.z * flip])

                data["A_cov"]["x"].append(
                    [t_sec, msg.linear_acceleration_covariance[0]]
                )
                data["A_cov"]["y"].append(
                    [t_sec, msg.linear_acceleration_covariance[4]]
                )
                data["A_cov"]["z"].append(
                    [t_sec, msg.linear_acceleration_covariance[8]]
                )

            elif connection.topic == topic_vel:
                # Account for DVL rotations in HoloOcean
                flip = -1 if SENSOR_INVERSIONS[auv_ns]["dvl"] else 1
                data["V"]["x"].append([t_sec, msg.twist.twist.linear.x])
                data["V"]["y"].append([t_sec, msg.twist.twist.linear.y * flip])
                data["V"]["z"].append([t_sec, msg.twist.twist.linear.z * flip])

                data["V_cov"]["x"].append([t_sec, msg.twist.covariance[0]])
                data["V_cov"]["y"].append([t_sec, msg.twist.covariance[4]])
                data["V_cov"]["z"].append([t_sec, msg.twist.covariance[8]])

            elif connection.topic == topic_orientation:
                data["Q"].append(
                    [
                        t_sec,
                        msg.orientation.w,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                    ]
                )

    for key in data:
        if isinstance(data[key], dict):
            for axis in data[key]:
                data[key][axis] = np.array(data[key][axis])
        else:
            data[key] = np.array(data[key])

    return data


def solve(bag_path):
    auv_ns = get_namespace(bag_path)
    d = get_data(bag_path, auv_ns)

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    t = d["V"]["x"][:, 0]
    Q_interp = np.zeros((len(t), 4))
    for i in range(4):
        Q_interp[:, i] = np.interp(t, d["Q"][:, 0], d["Q"][:, i + 1])

    w, x, y_q, z = Q_interp[:, 0], Q_interp[:, 1], Q_interp[:, 2], Q_interp[:, 3]

    # Find gravity vector (IMU frame)
    g_x_imu = 9.8 * 2.0 * (x * z - w * y_q)
    g_y_imu = 9.8 * 2.0 * (w * x + y_q * z)
    g_z_imu = 9.8 * (w**2 - x**2 - y_q**2 + z**2)
    flip = -1 if SENSOR_INVERSIONS[auv_ns]["imu"] else 1
    gravity_comp = {"x": g_x_imu, "y": g_y_imu * flip, "z": g_z_imu * flip}

    for idx, axis in enumerate(["x", "y", "z"]):
        print(f"\n--- Solving for {axis.upper()} Axis ---")

        # Interpolate to velocity timestamps (lowest freq ~ 20 Hz)
        if auv_ns == "blue0sim":
            # IMPORTANT! Fix for BlueROV2 ZOH forces (only publish on change)
            t_force = d["F"][axis][:, 0]
            f_force = d["F"][axis][:, 1]

            force_idx = np.searchsorted(t_force, t, side="right") - 1
            force_idx = np.clip(force_idx, 0, len(t_force) - 1)
            F = f_force[force_idx]
            F[t < t_force[0]] = 0.0

        else:
            F = np.interp(t, d["F"][axis][:, 0], d["F"][axis][:, 1])

        A_gravity = np.interp(t, d["A"][axis][:, 0], d["A"][axis][:, 1])
        V = d["V"][axis][:, 1]
        A_cov = np.interp(t, d["A_cov"][axis][:, 0], d["A_cov"][axis][:, 1])
        V_cov = d["V_cov"][axis][:, 1]

        # Subtract gravity
        A = A_gravity - gravity_comp[axis]

        # Filter out low-velocity data (needed?)
        mask = np.abs(V) > 0.001
        y_meas, a, v = F[mask], A[mask], V[mask]

        # Set up Weighted Linear Least Squares
        if ADD_COV_WEIGHTS:
            # Combine accel and vel covariances (could be better)
            weights = 1.0 / (A_cov[mask] + V_cov[mask] + 1e-6)
            W_sqrt = np.sqrt(weights)[:, np.newaxis]
        else:
            W_sqrt = np.ones((len(y_meas), 1))

        # Set up simplified Fossen equations
        # F = m*a + lin*v + quad*v|v|
        X_data = np.column_stack([a, v, v * np.abs(v)])
        X_w = X_data * W_sqrt
        y_w = y_meas[:, np.newaxis] * W_sqrt

        # Add Bayesian Priors
        W_prior = np.diag(1.0 / PRIOR_SIGMAS[axis])
        y_prior = (PRIOR_MEANS[axis] / PRIOR_SIGMAS[axis]).reshape(-1, 1)

        X_final = np.vstack([X_w, W_prior])
        y_final = np.vstack([y_w, y_prior])

        # SOLVE THE SYSTEM
        params, _, _, _ = np.linalg.lstsq(X_final, y_final.flatten(), rcond=None)

        print(f"Mass      : {params[0]:.4f}")
        print(f"Lin Drag  : {params[1]:.4f}")
        print(f"Quad Drag : {params[2]:.4f}")

        axes[idx].plot(t[mask], X_data @ params, label=f"Predicted {axis.upper()}")
        axes[idx].plot(t[mask], y_meas, alpha=0.7, label=f"Measured {axis.upper()}")
        axes[idx].set_ylabel(f"Force {axis.upper()} (N)")
        axes[idx].legend()

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: simple_sysid.py <bag_file>")
        sys.exit(1)
    solve(sys.argv[1])
