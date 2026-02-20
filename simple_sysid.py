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
import csv
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from functools import lru_cache
from scipy.interpolate import RegularGridInterpolator
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

# Simplified HoloOcean URDFs (and guessed BlueROV2 TFs -- check this)
SENSOR_INVERSIONS = {
    "coug0sim": {"imu": True, "dvl": True},
    "blue0sim": {"imu": True, "dvl": False},
    "bluerov2": {"imu": False, "dvl": False},
}

THRUST_DIRECTIONS = np.array(
    [
        [1 / np.sqrt(2), 1 / np.sqrt(2), 0],  # 0 Front Right (reversed)
        [1 / np.sqrt(2), -1 / np.sqrt(2), 0],  # 1 Front Left (reversed)
        [1 / np.sqrt(2), -1 / np.sqrt(2), 0],  # 2 Back Right
        [1 / np.sqrt(2), 1 / np.sqrt(2), 0],  # 3 Back Left
        [0, 0, 1],  # 4 Front Right Vertical (reversed)
        [0, 0, -1],  # 5 Front Left Vertical
        [0, 0, -1],  # 6 Back Right Vertical
        [0, 0, 1],  # 7 Back Left Vertical (reversed)
    ]
)

# --- T200 THRUSTER MAPPING ---


@lru_cache(None)
def _load_t200_table(voltage):
    pwm_vals, force_vals = [], []
    with open(f"data/T200-{voltage}V.csv", "r") as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        for row in reader:
            pwm_vals.append(float(row[0]))
            force_vals.append(float(row[5]))

    idx = np.argsort(pwm_vals)
    return np.array(pwm_vals)[idx], np.array(force_vals)[idx]


def get_force_interpolator(voltages=[10, 12, 14, 16, 18, 20]):
    try:
        force_rows, pwm_ref = [], None
        for v in sorted(voltages):
            pwm_vals, force_vals = _load_t200_table(v)
            if pwm_ref is None:
                pwm_ref = pwm_vals
            force_rows.append(force_vals * 9.81)  # Kg f -> Newtons

        return RegularGridInterpolator(
            (sorted(voltages), pwm_ref),
            np.vstack(force_rows),
            bounds_error=False,
            fill_value=0.0,
        )
    except FileNotFoundError:
        print("WARNING: data/T200 CSV files not found. Real-world parsing will fail.")
        return None


# --- MATH & PHYSICS HELPERS ---


def compute_gravity_components(q_w, q_x, q_y, q_z):
    g_x = 9.8 * 2.0 * (q_x * q_z - q_w * q_y)
    g_y = 9.8 * 2.0 * (q_w * q_x + q_y * q_z)
    g_z = 9.8 * (q_w**2 - q_x**2 - q_y**2 + q_z**2)
    return g_x, g_y, g_z


def zero_order_hold_interp(t_target, t_source, y_source):
    idx = np.clip(
        np.searchsorted(t_source, t_target, side="right") - 1, 0, len(t_source) - 1
    )
    y_target = y_source[idx]
    y_target[t_target < t_source[0]] = 0.0
    return y_target


def calculate_real_forces(t_pwm, pwms, t_volt, volts, interpolator):
    # Interpolate voltage to match PWM timestamps
    volt_interp = (
        np.interp(t_pwm, t_volt, volts)
        if len(t_volt) > 0
        else np.full(len(t_pwm), 16.0)
    )

    forces = []
    for pwm_row, volt in zip(pwms, volt_interp):
        f_body = np.zeros(3)
        for j, pwm in enumerate(pwm_row[: len(THRUST_DIRECTIONS)]):
            f_newton = interpolator((volt, np.clip(pwm, 1100, 1900)))
            f_body += f_newton * THRUST_DIRECTIONS[j]
        forces.append(f_body)

    return np.array(forces)


# --- DATA PARSING ---


def get_namespace(bag_path):
    name = Path(bag_path).name.lower()
    if "blue" in name:
        return "blue0sim"
    if "coug" in name:
        return "coug0sim"
    return "bluerov2"


def parse_rosbag(bag_path, auv_ns):
    print(f"Reading {bag_path} for {auv_ns}...")
    is_real = auv_ns == "bluerov2"

    # Topics are slightly diff between sim and the real world
    topics = {
        "accel": "/bluerov2/imu/data" if is_real else f"/{auv_ns}/imu/data_raw",
        "orient": f"/{auv_ns}/imu/data",
        "vel": f"/{auv_ns}/dvl/twist",
        "force": None if is_real else f"/{auv_ns}/cmd_wrench",
        "rcout": "/mavros/rc/out" if is_real else None,
        "battery": "/bluerov2/battery/status" if is_real else None,
    }
    active_topics = {t for t in topics.values() if t is not None}

    raw = {k: [] for k in ["A", "A_cov", "V", "V_cov", "Q", "F", "PWM", "VOLT"]}

    start_t = None
    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic in active_topics]
        for conn, t, rawdata in reader.messages(connections=connections):
            if start_t is None:
                start_t = t
            t_sec = (t - start_t) / 1e9
            msg = reader.deserialize(rawdata, conn.msgtype)
            topic = conn.topic

            if topic == topics["force"]:
                raw["F"].append(
                    [t_sec, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
                )

            if topic == topics["rcout"]:
                raw["PWM"].append([t_sec] + list(msg.channels[:8]))

            if topic == topics["battery"]:
                raw["VOLT"].append([t_sec, msg.voltage])

            if topic == topics["accel"]:
                raw["A"].append(
                    [
                        t_sec,
                        msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z,
                    ]
                )
                raw["A_cov"].append(
                    [
                        t_sec,
                        msg.linear_acceleration_covariance[0],
                        msg.linear_acceleration_covariance[4],
                        msg.linear_acceleration_covariance[8],
                    ]
                )

            if topic == topics["vel"]:
                v_lin = (
                    msg.twist.linear
                    if hasattr(msg.twist, "linear")
                    else msg.twist.twist.linear
                )
                cov = (
                    msg.twist.covariance
                    if hasattr(msg.twist, "covariance")
                    else [1.0] * 9
                )
                raw["V"].append([t_sec, v_lin.x, v_lin.y, v_lin.z])
                raw["V_cov"].append([t_sec, cov[0], cov[4], cov[8]])

            if topic == topics["orient"]:
                raw["Q"].append(
                    [
                        t_sec,
                        msg.orientation.w,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                    ]
                )

    data = {k: np.array(v) for k, v in raw.items() if len(v) > 0}

    # Get T200 forces from table
    if is_real and "PWM" in data:
        interpolator = get_force_interpolator()
        f_body = calculate_real_forces(
            data["PWM"][:, 0],
            data["PWM"][:, 1:],
            data.get("VOLT", np.empty((0, 2)))[:, 0],
            data.get("VOLT", np.empty((0, 2)))[:, 1],
            interpolator,
        )
        data["F"] = np.column_stack((data["PWM"][:, 0], f_body))

    return data


# --- SYSTEM IDENTIFICATION ---


def solve(bag_path):
    auv_ns = get_namespace(bag_path)
    d = parse_rosbag(bag_path, auv_ns)

    t = d["V"][:, 0]

    q_interp = np.column_stack(
        [np.interp(t, d["Q"][:, 0], d["Q"][:, i]) for i in range(1, 5)]
    )
    g_comps = compute_gravity_components(
        q_interp[:, 0], q_interp[:, 1], q_interp[:, 2], q_interp[:, 3]
    )

    imu_flip = -1 if SENSOR_INVERSIONS[auv_ns]["imu"] else 1
    dvl_flip = -1 if SENSOR_INVERSIONS[auv_ns]["dvl"] else 1

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    axis_names = ["x", "y", "z"]

    for idx, axis in enumerate(axis_names):
        print(f"\n--- Solving for {axis.upper()} Axis ---")
        if "F" not in d or len(d["F"]) == 0:
            print(f"Skipping {axis} (No force data)")
            continue

        flip_a = imu_flip if axis in ["y", "z"] else 1
        flip_v = dvl_flip if axis in ["y", "z"] else 1

        # Interpolate to velocity timestamps
        V = d["V"][:, idx + 1] * flip_v
        A_raw = np.interp(t, d["A"][:, 0], d["A"][:, idx + 1]) * flip_a
        A_cov = np.interp(t, d["A_cov"][:, 0], d["A_cov"][:, idx + 1])
        V_cov = d["V_cov"][:, idx + 1]

        if auv_ns in ["blue0sim", "bluerov2"]:
            # IMPORTANT! Fix for HoloOcean BlueROV2 ZOH forces (only publish on change)
            F = zero_order_hold_interp(t, d["F"][:, 0], d["F"][:, idx + 1])
        else:
            F = np.interp(t, d["F"][:, 0], d["F"][:, idx + 1])

        # Subtract gravity from raw accel
        A_clean = A_raw - (g_comps[idx] * flip_a)

        mask = np.abs(V) > 0.001
        y_meas, a, v = F[mask], A_clean[mask], V[mask]

        # Add covariances as weights
        W_sqrt = np.ones((len(y_meas), 1))
        if ADD_COV_WEIGHTS:
            A_cov_c = np.maximum(A_cov[mask], 0.0)
            V_cov_c = np.maximum(V_cov[mask], 0.0)
            W_sqrt = np.sqrt(1.0 / (A_cov_c + V_cov_c + 1e-6))[:, np.newaxis]

        X_data = np.column_stack([a, v, v * np.abs(v)])

        X_final = np.vstack([X_data * W_sqrt, np.diag(1.0 / PRIOR_SIGMAS[axis])])
        y_final = np.vstack(
            [
                y_meas[:, np.newaxis] * W_sqrt,
                (PRIOR_MEANS[axis] / PRIOR_SIGMAS[axis]).reshape(-1, 1),
            ]
        )

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
