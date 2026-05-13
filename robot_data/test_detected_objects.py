import numpy as np
import matplotlib.pyplot as plt

# Load saved map
data = np.load("occupancy_grid_map.npz")

grid = data["grid"]
resolution = data["resolution"]
origin_x = data["origin_x"]
origin_y = data["origin_y"]

print("Resolution is ", resolution)
print("Origin X is ", origin_x)
print("Origin Y is ", origin_y)
# Example detected objects
detected_objects = [
    {
        "class": "knife",
        "pose": [25, 15, -1.9242152627190454],
        "conf": 0.9,
        "count": 44,
    },
    {
        "class": "knife",
        "pose": [30, 22, -1.9124410468856199],
        "conf": 0.9,
        "count": 16,
    },
]
# Convert occupancy grid to displayable image
display_map = np.copy(grid)

# Unknown cells (-1) -> gray
display_map[display_map == -1] = 50

# Occupied cells (100) -> black
display_map[display_map == 100] = 0

# Free cells (0) -> white
display_map[display_map == 0] = 255

plt.figure(figsize=(10, 10))
plt.imshow(display_map, cmap="gray", origin="lower")

# Plot objects
for i in range(0, len(detected_objects), 1):

    world_x = detected_objects[i]["pose"][0]
    world_y = detected_objects[i]["pose"][1]

    # Convert world -> map coordinates
    map_x = int((world_x + origin_x) / resolution)
    map_y = int((world_y + origin_y) / resolution)
    print(f"Object {i}: World ({world_x:.2f}, {world_y:.2f}) -> Map ({map_x}, {map_y})")
    plt.scatter(map_x, map_y, s=120)

    plt.text(
        map_x + 2,
        map_y + 2,
        f"{detected_objects[i]['class']} ({detected_objects[i]['count']})",
        fontsize=10,
    )


plt.title("Detected Objects on Occupancy Grid")
plt.xlabel("Map X")
plt.ylabel("Map Y")

plt.show()
