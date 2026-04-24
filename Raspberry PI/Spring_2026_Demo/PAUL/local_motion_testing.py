import math
import random
import numpy as np
import matplotlib.pyplot as plt

from pathfinding import astar, inflate_obstacles, add_headings


# =========================
# SIM CONFIG
# =========================
GRID_W = 140
GRID_H = 100
CELL_SIZE_M = 0.10
ROBOT_RADIUS_M = 0.50

# Mode thresholds
R_MIN_CELLS = 6.0
ALPHA_MAX_DEG = 15.0
TURN_HEADING_DEG = 50.0

# For mecanum-style crab detection
CRAB_ANGLE_DEG = 15.0   # if motion direction differs this much from local heading, prefer crab

# Path cleanup
DENSIFY_STEP = 1
SMOOTH_WINDOW = 5


# =========================
# MECANUM MATRIX
# =========================
def mecanum_wheels(vx, vy, omega, L=0.3, W=0.3, r=0.05):
    M = np.array([
        [1, -1, -(L + W)],
        [1,  1,  (L + W)],
        [1,  1, -(L + W)],
        [1, -1,  (L + W)]
    ])
    v = np.array([vx, vy, omega])
    w = (1 / r) * M @ v
    return w


# =========================
# VISUALIZATION HELPERS
# =========================
def draw_headings(path):
    """
    Draw robot orientation arrows along the path.
    """
    waypoints = add_headings(path)

    for i in range(0, len(waypoints), 5):
        x, y, theta = waypoints[i]
        dx = math.cos(theta)
        dy = math.sin(theta)

        plt.arrow(
            x, y,
            dx, dy,
            head_width=0.6,
            head_length=0.8,
            fc='black',
            ec='black',
            alpha=0.7,
            length_includes_head=True,
            zorder=6
        )


def draw_icr(path):
    """
    Visualize Instantaneous Center of Rotation for curved regions.
    """
    waypoints = add_headings(path)

    for i in range(1, len(waypoints) - 1, 8):
        x, y, theta = waypoints[i]
        p0 = path[i - 1]
        p1 = path[i]
        p2 = path[i + 1]

        k = curvature_kappa(p0, p1, p2)
        if abs(k) < 1e-3:
            continue

        R = 1.0 / k
        icr_x = x - R * math.sin(theta)
        icr_y = y + R * math.cos(theta)

        plt.scatter(icr_x, icr_y, color="purple", s=20, alpha=0.6, zorder=4)


# =========================
# GRID WITH BIG OBSTACLES
# =========================
def make_big_obstacle_grid(w=GRID_W, h=GRID_H, n_obs=5):
    grid = np.zeros((h, w), dtype=np.uint8)

    for _ in range(n_obs):
        ow = random.randint(14, 28)
        oh = random.randint(14, 28)

        x0 = random.randint(10, w - ow - 10)
        y0 = random.randint(5, h - oh - 5)

        grid[y0:y0 + oh, x0:x0 + ow] = 1

    if random.random() < 0.5:
        y = random.randint(15, h - 15)
        grid[y:y + 3, 10:w - 10] = 1

    return grid


def pick_start_goal(grid):
    h, w = grid.shape

    for _ in range(2000):
        sy = random.randint(5, h - 6)
        gy = random.randint(5, h - 6)

        start = (3, sy)
        goal = (w - 4, gy)

        if grid[sy, 3] == 0 and grid[gy, w - 4] == 0:
            return start, goal

    return (3, h // 2), (w - 4, h // 2)


# =========================
# PATH DENSIFY + SMOOTH
# =========================
def densify_path(path, step=DENSIFY_STEP):
    if not path or len(path) < 2 or step <= 1:
        return path

    out = []
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        out.append((x1, y1))

        dx = x2 - x1
        dy = y2 - y1
        n = max(abs(dx), abs(dy)) * step

        if n > 0:
            for k in range(1, n):
                t = k / n
                out.append((x1 + t * dx, y1 + t * dy))

    out.append(path[-1])
    return out


def smooth_path_moving_average(path, window=SMOOTH_WINDOW):
    if not path or len(path) < window or window < 3 or window % 2 == 0:
        return path

    half = window // 2
    xs = np.array([p[0] for p in path], dtype=float)
    ys = np.array([p[1] for p in path], dtype=float)

    xs_s = xs.copy()
    ys_s = ys.copy()

    for i in range(half, len(path) - half):
        xs_s[i] = xs[i - half:i + half + 1].mean()
        ys_s[i] = ys[i - half:i + half + 1].mean()

    xs_s[0], ys_s[0] = xs[0], ys[0]
    xs_s[-1], ys_s[-1] = xs[-1], ys[-1]

    return list(zip(xs_s.tolist(), ys_s.tolist()))


# =========================
# GEOMETRY / MODE LOGIC
# =========================
def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def curvature_kappa(p0, p1, p2, eps=1e-9):
    def dist(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    a = dist(p0, p1)
    b = dist(p1, p2)
    c = dist(p0, p2)
    if a < eps or b < eps or c < eps:
        return 0.0

    x1, y1 = p1[0] - p0[0], p1[1] - p0[1]
    x2, y2 = p2[0] - p0[0], p2[1] - p0[1]
    cross = abs(x1 * y2 - y1 * x2)
    A = 0.5 * cross

    return (4.0 * A) / (a * b * c)


def merge_nearby_indices(indices, min_sep=5):
    if not indices:
        return []

    indices = sorted(indices)
    merged = [indices[0]]

    for i in indices[1:]:
        if i - merged[-1] >= min_sep:
            merged.append(i)

    return merged


def classify_drive_mode(path_xy, headings, i):
    """
    Mecanum-aware approximation:
    - ACKERMANN if local motion is mostly aligned with heading
    - CRAB if local motion is strongly sideways relative to heading
    """
    if i <= 0 or i >= len(path_xy) - 1:
        return "ACKERMANN"

    x0, y0 = path_xy[i - 1]
    x1, y1 = path_xy[i]
    x2, y2 = path_xy[i + 1]

    # average local direction of motion
    dx = x2 - x0
    dy = y2 - y0

    seg_angle = math.atan2(dy, dx)
    theta = headings[i]

    rel = wrap_pi(seg_angle - theta)
    rel_deg = abs(math.degrees(rel))

    if rel_deg >= CRAB_ANGLE_DEG:
        return "CRAB"

    return "ACKERMANN"


def annotate_modes(path_xy):
    """
    returns:
        drive_modes : ACKERMANN/CRAB along the path
        turn_points : indices where robot should pivot in place
    """
    n = len(path_xy)
    if n < 3:
        return ["ACKERMANN"] * n, []

    waypoints = add_headings(path_xy)
    headings = [wp[2] for wp in waypoints]

    turn_heading = math.radians(TURN_HEADING_DEG)

    drive_modes = ["ACKERMANN"] * n
    turn_candidates = []

    for i in range(1, n - 1):
        k = curvature_kappa(path_xy[i - 1], path_xy[i], path_xy[i + 1])
        R = float("inf") if abs(k) < 1e-6 else (1.0 / abs(k))
        dtheta = wrap_pi(headings[i + 1] - headings[i])

        # TURN = pivot event only
        if R < R_MIN_CELLS or abs(dtheta) > turn_heading:
            turn_candidates.append(i)

        # drive mode classification for non-pivot travel
        drive_modes[i] = classify_drive_mode(path_xy, headings, i)

    turn_points = merge_nearby_indices(turn_candidates)

    if n >= 2:
        drive_modes[0] = drive_modes[1]
        drive_modes[-1] = drive_modes[-2]

    return drive_modes, turn_points


# =========================
# PLOT HELPERS
# =========================
MODE_COLOR = {
    "ACKERMANN": "green",
    "CRAB": "orange",
    "TURN": "blue",
}


def plot_segmented_path(path_xy, modes):
    if len(path_xy) < 2:
        return

    seg_start = 0
    for i in range(1, len(path_xy)):
        if modes[i] != modes[i - 1]:
            xs = [p[0] for p in path_xy[seg_start:i + 1]]
            ys = [p[1] for p in path_xy[seg_start:i + 1]]
            plt.plot(xs, ys, color=MODE_COLOR[modes[i - 1]], linewidth=2.5, zorder=5)
            seg_start = i

    xs = [p[0] for p in path_xy[seg_start:]]
    ys = [p[1] for p in path_xy[seg_start:]]
    plt.plot(xs, ys, color=MODE_COLOR[modes[-1]], linewidth=2.5, zorder=5)


# =========================
# MAIN
# =========================
def main():
    grid = make_big_obstacle_grid()
    start, goal = pick_start_goal(grid)

    inflated = inflate_obstacles(grid, ROBOT_RADIUS_M, CELL_SIZE_M)
    path_grid = astar(inflated, start, goal)

    if path_grid is None or len(path_grid) < 2:
        print("No path found. Re-run (different random obstacles).")
        return

    path = [(float(x), float(y)) for (x, y) in path_grid]

    path = densify_path(path, step=DENSIFY_STEP)
    path = smooth_path_moving_average(path, window=SMOOTH_WINDOW)

    drive_modes, turn_points = annotate_modes(path)

    # ---- PLOT ----
    plt.figure(figsize=(10, 7))
    plt.imshow(grid, cmap="gray_r", origin="lower")
    plt.title("A* Path Annotated with Local Motion Modes")

    plot_segmented_path(path, drive_modes)
    draw_headings(path)
    draw_icr(path)

    plt.scatter([start[0]], [start[1]], s=140, marker="o",
                color="lime", edgecolors="black", linewidths=1.0, zorder=7)
    plt.scatter([goal[0]], [goal[1]], s=140, marker="o",
                color="red", edgecolors="black", linewidths=1.0, zorder=7)

    for idx in turn_points:
        x, y = path[idx]

        plt.scatter([x], [y],
                    s=100,
                    color="blue",
                    edgecolors="black",
                    zorder=8)

        plt.text(
            x, y,
            "TURN",
            fontsize=8,
            color="black",
            bbox=dict(facecolor="white", alpha=0.8, boxstyle="round,pad=0.2"),
            zorder=9
        )

    plt.plot([], [], color=MODE_COLOR["ACKERMANN"], label="ACKERMANN")
    plt.plot([], [], color=MODE_COLOR["CRAB"], label="CRAB")
    plt.scatter([], [], color="blue", edgecolors="black", label="TURN (pivot)", s=100)

    plt.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()