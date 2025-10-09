import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
import pandas as pd

# File paths
positions_file = "node_positions.csv"
pathloss_file = "loss_matrices.csv"  # Contains per-frame n x n matrices
zones_file = "zones.csv"
numberLinks = []

# Load node positions
def load_positions(file_path):
    positions = []
    with open(file_path, "r") as f:
        frame = []
        for line in f:
            line = line.strip()
            if line == "":
                if frame:
                    positions.append(frame)
                    frame = []
            else:
                x, y, z = map(float, line.split(","))
                frame.append((x, y, z))
    return positions

# Load pathloss matrices (one per frame)
def load_pathloss_matrices(file_path):
    """
    CSV format: Each block represents an n x n matrix separated by blank lines.
    """
    matrices = []
    current_matrix = []
    with open(file_path, "r") as f:
        for line in f:
            line = line.strip()
            if line == "":
                if current_matrix:
                    matrices.append(np.array(current_matrix))
                    current_matrix = []
            else:
                row = list(map(float, line.split(",")))
                current_matrix.append(row)
        if current_matrix:  # Append last matrix if not empty
            matrices.append(np.array(current_matrix))
    return matrices

# Load data
positions = load_positions(positions_file)
loss_matrices = load_pathloss_matrices(pathloss_file)
zones = pd.read_csv(zones_file)

print(f"Loaded {len(positions)} position frames")
print(f"Loaded {len(loss_matrices)} loss matrices")

# Plot setup
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-800, 800)
ax.set_ylim(-800, 800)
ax.set_title("Node Mobility and Links (Loss ≤ 100 shown in blue)")
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")

# Draw static rectangles from zone data
for _, row in zones.iterrows():
    width = row['xmax'] - row['xmin']
    height = row['ymax'] - row['ymin']
    rect = patches.Rectangle((row['xmin'], row['ymin']), width, height,
                           linewidth=1.5, edgecolor='red', facecolor='none', linestyle='--')
    ax.add_patch(rect)

scatter = ax.scatter([], [], c="black", s=50, label="Nodes")
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

# Updated plotting function
def update(frame_num):
    # Remove old lines (manually, since ax.lines.clear() doesn't exist)
    for line in ax.lines[:]:
        line.remove()
    
    current_positions = [(p[0], p[1]) for p in positions[frame_num]]
    scatter.set_offsets(current_positions)

    # Draw valid links (loss ≤ 100)
    loss_matrix = loss_matrices[frame_num]
    n = len(current_positions)
    numLink = 0
    for i in range(n):
        for j in range(i + 1, n):
            if loss_matrix[i][j] <= 100: # threshold for valid link
                numLink +=1
                line = plt.Line2D(
                    [current_positions[i][0], current_positions[j][0]],
                    [current_positions[i][1], current_positions[j][1]],
                    color='blue', linewidth=1.5, alpha=0.7
                )
                ax.add_line(line)
    numberLinks.append(numLink)

    time_text.set_text(f'Time: {frame_num * 0.2:.1f} s')
    return scatter, time_text, *ax.lines  # Return all drawn artists



# Animate
ani = animation.FuncAnimation(
    fig, update, frames=len(positions),
    interval=100, blit=False
)

ani.save("mobility_links_animation.gif", writer='pillow', fps=8)
plt.legend()
plt.show()

print(sum(numberLinks)/len(numberLinks))

frames = np.arange(len(numberLinks))
frames = frames * 0.2  # Convert to seconds

# Truncate arrays so that the last frame value is 65
max_frame = 55
mask = frames <= max_frame
frames = frames[mask]
numberLinks = np.array(numberLinks)[mask]

plt.plot(frames, numberLinks, "-b", label="Number of Links per Frame")
plt.ylim(0, 300)
plt.xlabel("Time (s)")
plt.ylabel("Number of Links")
plt.legend()
plt.show()
