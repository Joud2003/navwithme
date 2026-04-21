import numpy as np
import matplotlib.pyplot as plt

# Load the trajectory
data = np.loadtxt("trajectory.csv", delimiter=",")

# Separate x and y
x = data[:, 0]
y = data[:, 1]

# Plot
plt.figure(figsize=(6, 6))
plt.plot(x, y, marker="o", markersize=2, linestyle="-")
plt.title("Turtlebot3 Trajectory")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.grid(True)
plt.axis("equal")  # keep x and y scale the same
plt.show()
