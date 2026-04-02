import math
import numpy as np
import matplotlib.pyplot as plt

# pull in your real A* tools
from pathfinding import (
    astar,
    grid_to_world,
    smooth_path_spline,   # kept in case you want to experiment
    add_headings,
    inflate_obstacles,
)

# ============================================================
# PARAMETERS (matches ObstacleGrid)
# ============================================================
cell_size = 0.05      # 5 cm per cell — SAME AS obstacle_grid_processing
width_m   = 10
height_m  = 10

width  = int(width_m / cell_size)
height = int(height_m / cell_size)

# World origin is centered at (-5, -5) like your real SLAM grid
x_origin = -(width_m  / 2.0)   # -5.0
y_origin = -(height_m / 2.0)   # -5.0


# ============================================================
# Create empty occupancy grid (binary)
# ============================================================
grid = np.zeros((height, width), dtype=int)   # 0 = free, 1 = obstacle


# ============================================================
# WORLD → GRID conversion (MATCHES ObstacleGrid.world_to_grid)
# ============================================================
def world_to_grid(x_w, y_w):
    """
    Convert world coords (meters) to grid (row, col) indices.
    Mirrors the ObstacleGrid.world_to_grid behavior.
    NOTE: returns (row, col) = (iy, ix)
    """
    ix = int((x_w - x_origin) / cell_size)
    iy = height - 1 - int((y_w - y_origin) / cell_size)  # flip Y to match SLAM
    return (iy, ix)


def path_length(world_points):
    pts = np.array(world_points)
    diffs = np.diff(pts, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    return float(np.sum(seg_lengths))


def choose_n(world_points, resolution=0.02, min_n=5, max_n=600):
    L = path_length(world_points)
    print("Path length (m):", L)
    n = int(L / resolution)
    return max(min(n, max_n), min_n)


# ============================================================
# Insert manual fake obstacles
# ============================================================
def add_rect_obstacle(x1, y1, x2, y2):
    """
    Define a rectangular obstacle in WORLD coordinates:
        (x1, y1) bottom-left
        (x2, y2) top-right
    """
    iy1, ix1 = world_to_grid(x1, y1)
    iy2, ix2 = world_to_grid(x2, y2)
    iy_min, iy_max = sorted([iy1, iy2])
    ix_min, ix_max = sorted([ix1, ix2])
    grid[iy_min:iy_max+1, ix_min:ix_max+1] = 1
    print(f"Obstacle world=({x1},{y1})→({x2},{y2})  grid=rows[{iy_min}:{iy_max}], cols[{ix_min}:{ix_max}]")


# ============================================================
# Bezier smoothing
# ============================================================
def bezier(points, n=100):
    """
    Compute a Bezier curve through the given control points.
    points: list of (x, y) world points
    n: number of output points along the curve
    """
    pts = np.array(points, dtype=float)
    if len(pts) < 2:
        return pts

    N = len(pts) - 1
    t = np.linspace(0.0, 1.0, n)
    curve = np.zeros((n, 2), dtype=float)

    def C(n, k):
        return math.comb(n, k)

    for i in range(N + 1):
        bern = C(N, i) * (1 - t) ** (N - i) * (t ** i)
        curve += np.outer(bern, pts[i])

    return curve


# ============================================================
# 1) Add your fake world obstacles
# ============================================================
# Vertical wall with gap near center-right
add_rect_obstacle(-1,  1, -0.9,  4.9)
add_rect_obstacle(-1, -4.9, -0.9, 0)

add_rect_obstacle(1,  0, 1.1,  4.9)
add_rect_obstacle(1, -4.9, 1.1, -1)

#box
add_rect_obstacle(-4.9, -4.9, -4.8,  4.9)
add_rect_obstacle(4.9, 4.9, -4.8,  4.9)

add_rect_obstacle(4.9, 4.9, 4.8,  -4.9)
add_rect_obstacle(-4.9, -4.9, 4.8,  -4.9)


# Square block somewhere in the middle-left
add_rect_obstacle(2, 1, 3, 2)


# ============================================================
# 2) Pick START and GOAL in WORLD coords
# ============================================================
start_world = (-4.0, -4.0)
goal_world  = ( 4.0,  4.0)

start_rc = world_to_grid(*start_world)  # (row, col)
goal_rc  = world_to_grid(*goal_world)   # (row, col)

# A* expects (x, y) = (col, row)
start_A = (start_rc[1], start_rc[0])
goal_A  = (goal_rc[1],  goal_rc[0])

print("START (grid row,col):", start_rc)
print("GOAL  (grid row,col):", goal_rc)


# ============================================================
# 3) Inflate obstacles & run A*
# ============================================================
robot_radius = 0.40   # 40 cm safety radius
inflated_grid = inflate_obstacles(grid, robot_radius, cell_size)

path_grid = astar(inflated_grid, start_A, goal_A)

path_world = []
path_smooth = []
waypoints = []

if path_grid is None:
    print("❌ NO PATH FOUND — plotting grid anyway")
else:
    print("A* path grid length:", len(path_grid))

    # Convert → world
    path_world = grid_to_world(path_grid, (x_origin, y_origin), cell_size)

    # Smooth: Bezier over the world path
    if len(path_world) >= 3:
        n_dynamic = choose_n(path_world, resolution=0.02)
        print("Bezier sample count:", n_dynamic)
        path_smooth = bezier(path_world, n=n_dynamic)
    else:
        path_smooth = path_world

    # Headings (world frame, optional)
    waypoints = add_headings(path_smooth)


# ============================================================
# 4) Visualization (real vs inflated vs path)
# ============================================================
plt.figure(figsize=(6, 6))

# REAL OBSTACLES (black)
real_mask = (grid > 0)     # any positive means obstacle
plt.imshow(
    np.ma.masked_where(real_mask == 0, real_mask),
    cmap="gray",
    origin="lower",
    alpha=1.0,
)

# INFLATED OBSTACLES (blue), excluding real obstacle cells
inflated_mask = (inflated_grid == 1) & (grid == 0)
plt.imshow(
    np.ma.masked_where(inflated_mask == 0, inflated_mask),
    cmap="Blues",
    origin="lower",
    alpha=0.7,
)

# Start + Goal (remember x=col, y=row)
plt.scatter([start_A[0]], [start_A[1]], c="blue",  s=80, label="Start")
plt.scatter([goal_A[0]],  [goal_A[1]],  c="green", s=80, label="Goal")

# Raw A* path (on inflated grid indices)
if path_grid:
    px = [p[0] for p in path_grid]  # cols
    py = [p[1] for p in path_grid]  # rows
    plt.plot(px, py, "y.", markersize=3, label="Raw A* Path")

# Smooth Bezier path: convert world → grid (row,col) using SAME world_to_grid
if len(path_smooth) > 0:
    sx = [(p[0] - x_origin) / cell_size for p in path_smooth]
    sy = [(p[1] - y_origin) / cell_size for p in path_smooth]
    plt.plot(sx, sy, "r-", linewidth=2, label="Smooth Bezier Path")


plt.title("Real (Black) and Inflated (Blue) Obstacles")
plt.legend()
plt.tight_layout()
plt.show()
