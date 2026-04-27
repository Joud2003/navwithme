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
# CSV_FILE = "pose_log.csv"      # timestamp,x,y,theta
data = np.loadtxt("trajectory.csv", delimiter=",")

WAYPOINTS = [
    (0.0, 0.0),
    (2.0, 0.0),
    (2.0, -1.0),
    (0.0, -1.0),
]

TOLERANCE = 0.20  # meters


# -------------------------------------------------
# LOAD DATA
# -------------------------------------------------
x = data[:, 0]
y = data[:, 1]

# -------------------------------------------------
# FIND CLOSEST POSE TO EACH WAYPOINT
# -------------------------------------------------
results = []

for i, wp in enumerate(WAYPOINTS):
    wx, wy = wp

    dists = np.sqrt((x - wx) ** 2 + (y - wy) ** 2)

    best_idx = np.argmin(dists)
    best_dist = dists[best_idx]

    est_x = x[best_idx]
    est_y = y[best_idx]

    passed = best_dist <= TOLERANCE

    results.append(
        {
            "waypoint": i,
            "target_x": wx,
            "target_y": wy,
            "est_x": est_x,
            "est_y": est_y,
            "error_m": best_dist,
            "pass": passed,
        }
    )


# -------------------------------------------------
# PRINT RESULTS
# -------------------------------------------------
res_df = pd.DataFrame(results)
print(res_df)

mean_error = res_df["error_m"].mean()
max_error = res_df["error_m"].max()

print(f"\nMean waypoint error: {mean_error:.3f} m")
print(f"Max waypoint error : {max_error:.3f} m")


# -------------------------------------------------
# PLOT TRAJECTORY + WAYPOINTS
# -------------------------------------------------
plt.figure(figsize=(8, 6))

# robot path
plt.plot(x, y, label="Robot Path")

# target waypoints
wp_x = [p[0] for p in WAYPOINTS]
wp_y = [p[1] for p in WAYPOINTS]
plt.scatter(wp_x, wp_y, s=80, label="Waypoints")

# estimated hit points
plt.scatter(
    res_df["est_x"], res_df["est_y"], s=60, marker="x", label="Closest Reached Points"
)

for _, row in res_df.iterrows():
    plt.text(row["target_x"] + 0.03, row["target_y"] + 0.03, f"W{int(row['waypoint'])}")

plt.title("Waypoint Accuracy Validation")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
