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

# --- Braden's T200 Thruster Code ---

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


@lru_cache(None)
def _load_t200_table(voltage):
    pwm_vals, force_vals = [], []
    with open(f"data/T200-{voltage}V.csv", "r") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            pwm_vals.append(float(row[0]))
            force_vals.append(float(row[5]))

    idx = np.argsort(pwm_vals)
    return np.array(pwm_vals)[idx], np.array(force_vals)[idx]


def build_t200_surface(voltages):
    voltages = np.array(sorted(voltages))
    force_rows, pwm_reference = [], None

    for V in voltages:
        pwm_vals, force_vals = _load_t200_table(V)
        if pwm_reference is None:
            pwm_reference = pwm_vals
        force_rows.append(force_vals * 9.81)  # Kg f -> Newtons

    return voltages, pwm_reference, np.vstack(force_rows)


try:
    _v, _p, _f_table = build_t200_surface([10, 12, 14, 16, 18, 20])
    force_interpolator = RegularGridInterpolator(
        (_v, _p), _f_table, bounds_error=False, fill_value=0.0
    )
except FileNotFoundError:
    print("WARNING: data/T200 CSV files not found. Real-world parsing will fail.")
    force_interpolator = None


def interpolate_force(pwm, voltage):
    pwm = np.clip(pwm, 1100, 1900)
    return force_interpolator((voltage, pwm))


# --- End of Braden's T200 Thruster Code ---


def get_namespace(bag_path):
    name = Path(bag_path).name.lower()
    if "blue" in name:
        return "blue0sim"
    elif "coug" in name:
        return "coug0sim"
    else:
        return "bluerov2"


def get_data(bag_path, auv_ns):
    print(f"Reading {bag_path} for {auv_ns}...")
    is_real = auv_ns == "bluerov2"

    # Topics are slightly diff in sim and real world
    t_accel = "/bluerov2/imu/data" if is_real else f"/{auv_ns}/imu/data_raw"
    t_orient = f"/{auv_ns}/imu/data"
    t_vel = f"/{auv_ns}/dvl/twist"
    t_force = None if is_real else f"/{auv_ns}/cmd_wrench"
    t_rcout = "/mavros/rc/out" if is_real else None
    t_battery = "/bluerov2/battery/status" if is_real else None

    target_topics = {t_accel, t_orient, t_vel, t_force, t_rcout, t_battery} - {None}

    data = {
        "F": {"x": [], "y": [], "z": []},
        "A": {"x": [], "y": [], "z": []},
        "V": {"x": [], "y": [], "z": []},
        "A_cov": {"x": [], "y": [], "z": []},
        "V_cov": {"x": [], "y": [], "z": []},
        "Q": [],
        "PWM": [],
        "VOLT": [],
    }

    start_t = None
    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic in target_topics]

        for connection, t, rawdata in reader.messages(connections=connections):
            if start_t is None:
                start_t = t
            t_sec = (t - start_t) / 1e9
            msg = reader.deserialize(rawdata, connection.msgtype)
            topic = connection.topic

            # Calculated sim wrench
            if topic == t_force:
                for ax in ["x", "y", "z"]:
                    data["F"][ax].append([t_sec, getattr(msg.wrench.force, ax)])

            # Real-world PWM
            elif topic == t_rcout:
                data["PWM"].append([t_sec] + list(msg.channels[:8]))
            elif topic == t_battery:
                data["VOLT"].append([t_sec, msg.voltage])

            # IMU acceleration
            if topic == t_accel:
                flip = -1 if SENSOR_INVERSIONS[auv_ns]["imu"] else 1
                data["A"]["x"].append([t_sec, msg.linear_acceleration.x])
                data["A"]["y"].append([t_sec, msg.linear_acceleration.y * flip])
                data["A"]["z"].append([t_sec, msg.linear_acceleration.z * flip])

                cov = msg.linear_acceleration_covariance
                data["A_cov"]["x"].append([t_sec, cov[0]])
                data["A_cov"]["y"].append([t_sec, cov[4]])
                data["A_cov"]["z"].append([t_sec, cov[8]])

            # DVL velocity
            if topic == t_vel:
                flip = -1 if SENSOR_INVERSIONS[auv_ns]["dvl"] else 1
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

                data["V"]["x"].append([t_sec, v_lin.x])
                data["V"]["y"].append([t_sec, v_lin.y * flip])
                data["V"]["z"].append([t_sec, v_lin.z * flip])

                data["V_cov"]["x"].append([t_sec, cov[0]])
                data["V_cov"]["y"].append([t_sec, cov[4]])
                data["V_cov"]["z"].append([t_sec, cov[8]])

            # Orientation (gravity)
            if topic == t_orient:
                data["Q"].append(
                    [
                        t_sec,
                        msg.orientation.w,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                    ]
                )

    # --- Braden's T200 Thruster Code ---

    if is_real and data["PWM"]:
        pwm_arr = np.array(data["PWM"])
        volt_arr = np.array(data["VOLT"])

        t_pwm = pwm_arr[:, 0]
        pwms = pwm_arr[:, 1:]
        volts = (
            np.interp(t_pwm, volt_arr[:, 0], volt_arr[:, 1])
            if len(volt_arr) > 0
            else np.full(len(t_pwm), 16.0)
        )

        for t, pwm_row, volt in zip(t_pwm, pwms, volts):
            force_body = np.zeros(3)
            for j in range(min(len(pwm_row), len(THRUST_DIRECTIONS))):
                f_newton = interpolate_force(pwm_row[j], volt)
                force_body += f_newton * THRUST_DIRECTIONS[j]

            data["F"]["x"].append([t, force_body[0]])
            data["F"]["y"].append([t, force_body[1]])
            data["F"]["z"].append([t, force_body[2]])

    # --- End of Braden's T200 Thruster Code ---

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

    # Interpolate orientation to velocity timestamps
    Q_interp = np.column_stack(
        [np.interp(t, d["Q"][:, 0], d["Q"][:, i]) for i in range(1, 5)]
    )
    w, x, y_q, z = Q_interp[:, 0], Q_interp[:, 1], Q_interp[:, 2], Q_interp[:, 3]

    # Find gravity vector (IMU frame)
    g_x_imu = 9.8 * 2.0 * (x * z - w * y_q)
    g_y_imu = 9.8 * 2.0 * (w * x + y_q * z)
    g_z_imu = 9.8 * (w**2 - x**2 - y_q**2 + z**2)

    flip = -1 if SENSOR_INVERSIONS[auv_ns]["imu"] else 1
    gravity_comp = {"x": g_x_imu, "y": g_y_imu * flip, "z": g_z_imu * flip}

    for idx, axis in enumerate(["x", "y", "z"]):
        print(f"\n--- Solving for {axis.upper()} Axis ---")

        if len(d["F"][axis]) == 0:
            print(f"Skipping {axis} (No force data)")
            continue

        # Interpolate to velocity timestamps
        if auv_ns in ["blue0sim", "bluerov2"]:
            # IMPORTANT! Fix for HoloOcean BlueROV2 ZOH forces (only publish on change)
            t_force, f_force = d["F"][axis][:, 0], d["F"][axis][:, 1]
            force_idx = np.clip(
                np.searchsorted(t_force, t, side="right") - 1, 0, len(t_force) - 1
            )
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
            # Fix for invalid covariances
            A_cov_clean = np.maximum(A_cov[mask], 0.0)
            V_cov_clean = np.maximum(V_cov[mask], 0.0)
            W_sqrt = np.sqrt(1.0 / (A_cov_clean + V_cov_clean + 1e-6))[:, np.newaxis]
        else:
            W_sqrt = np.ones((len(y_meas), 1))

        # Set up simplified Fossen equations
        # F = m*a + lin*v + quad*v|v|
        X_data = np.column_stack([a, v, v * np.abs(v)])
        X_w = X_data * W_sqrt
        y_w = y_meas[:, np.newaxis] * W_sqrt

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
