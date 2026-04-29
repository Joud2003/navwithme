# # Load the trajectory
# data = np.loadtxt("trajectory.csv", delimiter=",")

# # Separate x and y
# x = data[:, 0]
# y = data[:, 1]

# # Plot
# plt.figure(figsize=(6, 6))
# plt.plot(x, y, marker="o", markersize=2, linestyle="-")
# plt.title("Turtlebot3 Trajectory")
# plt.xlabel("X position (m)")
# plt.ylabel("Y position (m)")
# plt.grid(True)
# plt.axis("equal")  # keep x and y scale the same
# plt.show()
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# -------------------------------------------------
# CONFIG
# -------------------------------------------------
CSV_FILE = "trajectories.csv"  # traj_x,traj_y,traj_theta,gt_x,gt_y,gt_theta


# -------------------------------------------------
# LOAD DATA
# -------------------------------------------------
raw = pd.read_csv(CSV_FILE)

x = raw["traj_x"].to_numpy()[: 610]
y = raw["traj_y"].to_numpy()[: 610]
theta = raw["traj_theta"].to_numpy()[: 610]
gt_x = raw["gt_x"].to_numpy()[: 610]
gt_y = raw["gt_y"].to_numpy()[: 610]
gt_theta = raw["gt_theta"].to_numpy()[: 610]


# -------------------------------------------------
# VALIDATE ESTIMATED POSE AGAINST GROUND TRUTH
# -------------------------------------------------
def wrap_angle_diff(angle, reference):
    diff = angle - reference
    return (diff + np.pi) % (2 * np.pi) - np.pi


pose_error = np.sqrt((x - gt_x) ** 2 + (y - gt_y) ** 2)
theta_error = np.abs(wrap_angle_diff(theta, gt_theta))

validation_df = pd.DataFrame(
    {
        "traj_x": x,
        "traj_y": y,
        "traj_theta": theta,
        "gt_x": gt_x,
        "gt_y": gt_y,
        "gt_theta": gt_theta,
        "pos_error_m": pose_error,
        "theta_error_rad": theta_error,
    }
)

print(validation_df[["pos_error_m", "theta_error_rad"]].describe())
print(f"\nMean pos error  : {pose_error.mean():.3f} m")
print(f"Max pos error   : {pose_error.max():.3f} m")
print(f"Mean theta error: {(theta_error).mean():.3f} rad")
print(f"Max theta error : {(theta_error).max():.3f} rad")


# -------------------------------------------------
# PLOT TRAJECTORY COMPARISON
# -------------------------------------------------
plt.figure(figsize=(8, 6))

plt.plot(gt_x, gt_y, linestyle="--", color="gray", label="Ground Truth Path")
plt.plot(x, y, label="Estimated Path", linewidth=2)

plt.scatter(x[0], y[0], color="green", s=80, marker="o", label="Start")
plt.scatter(x[-1], y[-1], color="red", s=80, marker="X", label="End")

plt.title("Estimated Trajectory vs Ground Truth")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
