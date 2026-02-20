import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader

ADD_COV_WEIGHTS = True

TOPIC_FORCE = "/coug0sim/cmd_wrench"
TOPIC_ACCEL = "/coug0sim/imu/data_raw"
TOPIC_VEL = "/coug0sim/dvl/data"
TOPIC_ORIENTATION = "/coug0sim/imu/data"

PRIOR_MEANS = np.array([31.87, 1.59, 6.11])
PRIOR_SIGMAS = np.array([50.0, 50.0, 50.0])


def get_data(bag_path):
    print(f"Reading {bag_path}...")
    data = {"t": [], "F": [], "A": [], "V": [], "A_cov": [], "V_cov": [], "P": []}

    start_t = None
    with AnyReader([Path(bag_path)]) as reader:
        connections = [
            x
            for x in reader.connections
            if x.topic in [TOPIC_FORCE, TOPIC_ACCEL, TOPIC_VEL, TOPIC_ORIENTATION]
        ]

        for connection, t, rawdata in reader.messages(connections=connections):
            if start_t is None:
                start_t = t
            t_sec = (t - start_t) / 1e9

            msg = reader.deserialize(rawdata, connection.msgtype)

            if connection.topic == TOPIC_FORCE:
                data["F"].append([t_sec, msg.wrench.force.x])
            elif connection.topic == TOPIC_ACCEL:
                data["A"].append([t_sec, msg.linear_acceleration.x])
                data["A_cov"].append([t_sec, msg.linear_acceleration_covariance[0]])
            elif connection.topic == TOPIC_VEL:
                data["V"].append([t_sec, msg.velocity.x])
                data["V_cov"].append([t_sec, msg.covariance[0]])
            elif connection.topic == TOPIC_ORIENTATION:
                w, x, y, z = (
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                )
                sinp = 2.0 * (w * y - z * x)
                pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
                data["P"].append([t_sec, pitch])

    return {k: np.array(v) for k, v in data.items()}


def solve(bag_path):
    d = get_data(bag_path)

    # Interpolate to match velocity timestamps
    t = d["V"][:, 0]
    F = np.interp(t, d["F"][:, 0], d["F"][:, 1])
    A_gravity = np.interp(t, d["A"][:, 0], d["A"][:, 1])
    V = d["V"][:, 1]
    P = np.interp(t, d["P"][:, 0], d["P"][:, 1])
    A_cov = np.interp(t, d["A_cov"][:, 0], d["A_cov"][:, 1])
    V_cov = d["V_cov"][:, 1]

    # Subtract gravity component (ENU, upside-down IMU)
    A = A_gravity + 9.8 * np.sin(P)

    # Filter out low-velocity data
    mask = np.abs(V) > 0.001
    y, a, v = F[mask], A[mask], V[mask]

    # Handle covariance weighting
    if ADD_COV_WEIGHTS:
        weights = 1.0 / (A_cov[mask] + V_cov[mask] + 1e-6)
        W_sqrt = np.sqrt(weights)[:, np.newaxis]
    else:
        W_sqrt = np.ones((len(y), 1))

    # F = m*a + lin*v + quad*v|v|
    X_data = np.column_stack([a, v, v * np.abs(v)])
    X_w = X_data * W_sqrt
    y_w = y[:, np.newaxis] * W_sqrt

    # Add Bayesian Priors
    W_prior = np.diag(1.0 / PRIOR_SIGMAS)
    y_prior = (PRIOR_MEANS / PRIOR_SIGMAS).reshape(-1, 1)

    X_final = np.vstack([X_w, W_prior])
    y_final = np.vstack([y_w, y_prior])

    params, _, _, _ = np.linalg.lstsq(X_final, y_final.flatten(), rcond=None)

    print(f"Mass      : {params[0]:.4f}")
    print(f"Lin Drag  : {params[1]:.4f}")
    print(f"Quad Drag : {params[2]:.4f}")

    plt.plot(t[mask], X_data @ params, label="Predicted Force")
    plt.plot(t[mask], y, label="Measured Force")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    solve(sys.argv[1])
