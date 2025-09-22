import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# ---------------- PARAMETERS ----------------
x_min, x_max = -5, 5
y_min, y_max = -5, 5
tree_box_size = 2
num_trees = 20
drone_clearance = 1.3  # corridor width for drone

# ---------------- ZIG-ZAG PATH ----------------
waypoints = [
    (0, 0),
    (2, -2),
    (2, -4),
    (0, -4),
    (0, -8),
    (8, -8)
]

# ---------------- TREE PLACEMENT ----------------
selected_positions = []
attempts = 0
max_attempts = 10000

while len(selected_positions) < num_trees and attempts < max_attempts:
    attempts += 1
    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    
    # Round x and y to the first decimal place
    x = round(x, 1)
    y = round(y, 1)
    
    # Check distance to all path segments
    # Calculate min_dist_to_path using the rounded coordinates
    min_dist_to_path = min([np.hypot(x-wx, y-wy) for wx, wy in waypoints])
    if min_dist_to_path < drone_clearance + tree_box_size/2:
        continue
    
    # Ensure no overlap with existing trees using rounded coordinates
    if all(np.hypot(x-tx, y-ty) >= tree_box_size for tx, ty in selected_positions):
        selected_positions.append((x, y))

# ---------------- PRINT XML FOR GAZEBO ----------------
print("<!-- Generated XML for Gazebo models -->")
for i, (x, y) in enumerate(selected_positions):
    # Print positions rounded to two decimal places for XML clarity
    print(f"""  <include>
    <uri>model://Pine Tree</uri>
    <name>pine_tree_{i+1}</name>
    <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
  </include>""")

# ---------------- PRINT WAYPOINTS ----------------
print("\nWaypoints for drone to follow:")
for i, (x, y) in enumerate(waypoints):
    print(f"Waypoint {i+1}: x={x:.2f}, y={y:.2f}")

# ---------------- PLOTTING ----------------
fig, ax = plt.subplots(figsize=(10, 8)) # Make plot a bit larger for better visibility
ax.set_aspect('equal')

# Plot trees
for (x, y) in selected_positions:
    # Use rounded positions for plotting
    rect = patches.Rectangle(
        (x - tree_box_size/2, y - tree_box_size/2),
        tree_box_size, tree_box_size,
        linewidth=1, edgecolor='green', facecolor='none',
        label='_nolegend_' # Prevent duplicate legend entries
    )
    ax.add_patch(rect)
    # Add a point at the exact center of the tree for clarity
    ax.plot(x, y, 'g.', markersize=5, label='_nolegend_')


# Plot path and waypoints
way_x, way_y = zip(*waypoints)
ax.plot(way_x, way_y, 'b--', linewidth=2, label='Drone Path')
ax.scatter(way_x, way_y, c='blue', marker='o', s=80, zorder=5, label='Waypoints') # Increase size and zorder for visibility

# Drone size at start
drone_diameter = 1.3
origin_circle = patches.Circle(
    waypoints[0],
    radius=drone_diameter/2,
    edgecolor='red',
    facecolor='none',
    linewidth=2,
    label='Drone Start Clearance'
)
ax.add_patch(origin_circle)

# Set grid and labels
ax.grid(True, linestyle=':', alpha=0.7)
ax.set_xlim(x_min-2, x_max+2)
ax.set_ylim(y_min-2, y_max+2)
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("Forest Layout with Zig-Zag Drone Path and Rounded Tree Positions")
plt.legend()
plt.show()