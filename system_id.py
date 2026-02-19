import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader

TOPIC_FORCE = "/coug0sim/cmd_wrench"
TOPIC_ACCEL = "/coug0sim/imu/data_raw"
TOPIC_VEL = "/coug0sim/dvl/data"

PRIOR_MEANS = np.array([32.0, 0.0, 0.0])
PRIOR_SIGMAS = np.array([1e-9, 50.0, 50.0])


def get_data(bag_path):
    print(f"Reading {bag_path}...")
    data = {"t": [], "F": [], "A": [], "V": [], "CovA": [], "CovV": []}

    start_t = None
    with AnyReader([Path(bag_path)]) as reader:
        connections = [
            x
            for x in reader.connections
            if x.topic in [TOPIC_FORCE, TOPIC_ACCEL, TOPIC_VEL]
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
                data["CovA"].append([t_sec, msg.linear_acceleration_covariance[0]])
            elif connection.topic == TOPIC_VEL:
                data["V"].append([t_sec, msg.velocity.x])
                data["CovV"].append([t_sec, msg.covariance[0]])

    return {k: np.array(v) for k, v in data.items()}


def solve(bag_path):
    d = get_data(bag_path)

    # Interpolate to match velocity timestamps
    t = d["V"][:, 0]
    F = np.interp(t, d["F"][:, 0], d["F"][:, 1])
    A = np.interp(t, d["A"][:, 0], d["A"][:, 1])
    V = d["V"][:, 1]

    CovA = np.interp(t, d["CovA"][:, 0], d["CovA"][:, 1])
    CovV = d["CovV"][:, 1]

    # Filter out low-velocity data
    mask = np.abs(V) > 0.05
    y, a, v = F[mask], A[mask], V[mask]

    # Set weights based on covariance
    weights = 1.0 / (CovA[mask] + CovV[mask] + 1e-6)
    W_sqrt = np.sqrt(weights)[:, np.newaxis]

    # F = m*a + lin*v + quad*v|v|
    X_data = np.column_stack([a, v, v * np.abs(v)])
    X_w = X_data * W_sqrt
    y_w = y[:, np.newaxis] * W_sqrt

    # Add Bayesian Priors
    W_prior = np.diag(1.0 / PRIOR_SIGMAS)
    y_prior = (PRIOR_MEANS / PRIOR_SIGMAS).reshape(-1, 1)

    X_final = np.vstack([X_w, W_prior])
    y_final = np.vstack([y_w, y_prior])
    params = np.linalg.lstsq(X_final, y_final, rcond=None)[0].flatten()

    print(f"Mass      : {params[0]:.4f}")
    print(f"Lin Drag  : {params[1]:.4f}")
    print(f"Quad Drag : {params[2]:.4f}")

    plt.plot(t[mask], X_data @ params, label="Predicted Force")
    plt.plot(t[mask], y, label="Measured Force")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    solve(sys.argv[1])
