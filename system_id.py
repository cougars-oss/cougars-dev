import sys
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from dvl_msgs.msg import DVL

TOPIC_FORCE = "/coug0sim/cmd_wrench"
TOPIC_ACCEL = "/coug0sim/imu/data_raw"
TOPIC_VEL = "/coug0sim/dvl/data"

PRIOR_MEANS = np.array([32.0, 0.0, 0.0])
PRIOR_SIGMAS = np.array([1e-9, 50.0, 50.0])


def get_data(bag_path):
    print(f"Reading {bag_path}...")
    data = {"t": [], "F": [], "A": [], "V": [], "CovA": [], "CovV": []}

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    start_t = None
    while reader.has_next():
        topic, msg_data, t = reader.read_next()
        if start_t is None:
            start_t = t
        t_sec = (t - start_t) / 1e9

        if topic == TOPIC_FORCE:
            msg = deserialize_message(msg_data, WrenchStamped)
            data["F"].append([t_sec, msg.wrench.force.x])
        elif topic == TOPIC_ACCEL:
            msg = deserialize_message(msg_data, Imu)
            data["A"].append([t_sec, msg.linear_acceleration.x])
            data["CovA"].append([t_sec, msg.linear_acceleration_covariance[0]])
        elif topic == TOPIC_VEL:
            msg = deserialize_message(msg_data, DVL)
            data["V"].append([t_sec, msg.velocity.x])
            data["CovV"].append([t_sec, msg.covariance[0]])

    return {k: np.array(v) for k, v in data.items()}


def solve(bag_path):
    d = get_data(bag_path)

    # Interpolate to match accel timestamps
    t = d["A"][:, 0]
    F = np.interp(t, d["F"][:, 0], d["F"][:, 1])
    V = np.interp(t, d["V"][:, 0], d["V"][:, 1])
    A = d["A"][:, 1]

    CovV = np.interp(t, d["CovV"][:, 0], d["CovV"][:, 1])
    CovA = d["CovA"][:, 1]

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
