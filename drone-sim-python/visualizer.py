import io
import base64
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def render_trajectory_3d(trajectory_tensor: np.ndarray) -> str:
    num_drones, num_timesteps, _ = trajectory_tensor.shape
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    colors = plt.cm.tab10(np.linspace(0, 1, num_drones))

    for i in range(num_drones):
        traj = trajectory_tensor[i]
        ax.plot(
            traj[:, 0], traj[:, 1], traj[:, 2],
            color=colors[i], linewidth=1.5, alpha=0.7,
            label=f"Drone-{i}"
        )
        ax.scatter(
            traj[0, 0], traj[0, 1], traj[0, 2],
            color=colors[i], marker="o", s=60, edgecolors="black"
        )
        ax.scatter(
            traj[-1, 0], traj[-1, 1], traj[-1, 2],
            color=colors[i], marker="*", s=100, edgecolors="black"
        )

    max_range = np.array([
        trajectory_tensor[:, :, 0].ptp(),
        trajectory_tensor[:, :, 1].ptp(),
        trajectory_tensor[:, :, 2].ptp(),
    ]).max() / 2.0
    mid_x = trajectory_tensor[:, :, 0].mean()
    mid_y = trajectory_tensor[:, :, 1].mean()
    mid_z = trajectory_tensor[:, :, 2].mean()
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Drone Formation Trajectory (3D)")
    ax.legend(loc="upper left", fontsize=8, ncol=2)
    plt.tight_layout()

    buf = io.BytesIO()
    plt.savefig(buf, format="png", dpi=120)
    buf.seek(0)
    img_b64 = base64.b64encode(buf.read()).decode("utf-8")
    plt.close(fig)
    return img_b64


def render_top_view(trajectory_tensor: np.ndarray) -> str:
    num_drones, num_timesteps, _ = trajectory_tensor.shape
    fig, ax = plt.subplots(figsize=(10, 8))
    colors = plt.cm.tab10(np.linspace(0, 1, num_drones))

    for i in range(num_drones):
        traj = trajectory_tensor[i]
        ax.plot(
            traj[:, 0], traj[:, 1],
            color=colors[i], linewidth=1.5, alpha=0.7,
            label=f"Drone-{i}"
        )
        ax.scatter(
            traj[0, 0], traj[0, 1],
            color=colors[i], marker="o", s=60, edgecolors="black"
        )
        ax.scatter(
            traj[-1, 0], traj[-1, 1],
            color=colors[i], marker="*", s=100, edgecolors="black"
        )
    ax.set_aspect("equal")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Drone Formation Trajectory (Top View)")
    ax.legend(loc="upper left", fontsize=8, ncol=2)
    plt.tight_layout()

    buf = io.BytesIO()
    plt.savefig(buf, format="png", dpi=120)
    buf.seek(0)
    img_b64 = base64.b64encode(buf.read()).decode("utf-8")
    plt.close(fig)
    return img_b64
