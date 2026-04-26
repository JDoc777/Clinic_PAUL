import random
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
from pyqtgraph.Qt import QtCore

from pathfinding import astar, inflate_obstacles, bezier


# =========================
# SIM CONFIG (ALL METRIC)
# =========================
MAP_W_M = 5.0
MAP_H_M = 5.0
CELL_SIZE_M = 0.10

GRID_W = int(round(MAP_W_M / CELL_SIZE_M))
GRID_H = int(round(MAP_H_M / CELL_SIZE_M))

ROBOT_RADIUS_M = 0.25
SLOPE_LENGTH_M = 0.50

# =========================
# MOTION PARAMETERS (METERS)
# =========================
BOT_SPEED_MPS = 0.10
BOT_ACCEL = 0.04
BOT_DECEL = 0.08

TIP_TURN_RATE = 0.06
TIP_STOP_TOL = np.radians(1.0)

LOOKAHEAD_DISTANCE_M = 0.20
TIP_SLOW_RADIUS_M = 0.40
TIP_STOP_RADIUS_M = 0.10

FRAME_DELAY_FAST_MS = 20
FRAME_DELAY_MED_MS = 50
FRAME_DELAY_SLOW_MS = 100
TIP_DELAY_MS = 20
GRAPH_DELAY_MS = 20

R_MIN_M = 1.5
DENSIFY_STEP_M = 0.02
POINT_MATCH_TOL_M = 0.03

# robustness
HEADING_LOOKAHEAD_PTS = 5
MAX_HEADING_JUMP_RAD = np.radians(120)

# ==========================================
# USER GOAL HEADING
# ==========================================
GOAL_FINAL_THETA_DEG = 270.0   # <-- change this

# ==========================================
# MODE DECISION TUNING
# ==========================================
ACK_MAX_ERR_RAD = np.radians(15.0)
TIP_TRIGGER_ERR_RAD = np.radians(45.0)

TIP_LATERAL_RATIO = 1.6
TIP_LATERAL_STREAK = 5

SEGMENT_HEADING_LOOKAHEAD_PTS = 10

PRESERVE_HEADING_GAIN = 0.25
MIN_TRANS_COMPONENT = 1e-6


# =========================
# HELPERS
# =========================
def angle_wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def grid_to_world_point(pt):
    x, y = pt
    return np.array([x * CELL_SIZE_M, y * CELL_SIZE_M], dtype=float)


def world_to_grid_point(pt):
    x, y = pt
    gx = int(round(x / CELL_SIZE_M))
    gy = int(round(y / CELL_SIZE_M))
    gx = max(0, min(GRID_W - 1, gx))
    gy = max(0, min(GRID_H - 1, gy))
    return gx, gy


def densify_path(path, step=DENSIFY_STEP_M):
    path = np.asarray(path, dtype=float)
    if len(path) == 0:
        return path
    if len(path) == 1:
        return path.copy()

    dense = [path[0]]

    for i in range(len(path) - 1):
        p0 = np.array(path[i], dtype=float)
        p1 = np.array(path[i + 1], dtype=float)

        d = np.linalg.norm(p1 - p0)
        if d < 1e-12:
            continue

        n = max(1, int(np.ceil(d / step)))

        for k in range(1, n + 1):
            t = k / n
            p = p0 + t * (p1 - p0)
            dense.append(p)

    return np.array(dense, dtype=float)


def mecanum_general(vx, vy, omega, r=0.04, Lx=0.15, Ly=0.15):
    k = Lx + Ly

    M = np.array([
        [1, -1, -k],
        [1,  1,  k],
        [1,  1, -k],
        [1, -1,  k]
    ], dtype=float)

    V = np.array([vx, vy, omega], dtype=float)
    wheels = (1.0 / r) * (M @ V)

    MAX_WHEEL_SPEED = 31.42
    wheels = np.clip(wheels, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    return wheels


def mecanum_ack(vx, omega, r=0.04, Lx=0.15, Ly=0.15):
    return mecanum_general(vx, 0.0, omega, r=r, Lx=Lx, Ly=Ly)


def mecanum_preserve(vx, vy, omega, r=0.04, Lx=0.15, Ly=0.15):
    return mecanum_general(vx, vy, omega, r=r, Lx=Lx, Ly=Ly)


def mecanum_tip(omega, r=0.04, Lx=0.15, Ly=0.15):
    return mecanum_general(0.0, 0.0, omega, r=r, Lx=Lx, Ly=Ly)


def slope_intersection(p1, m1, p2, m2):
    x1, y1 = p1
    x2, y2 = p2

    if np.isinf(m1) and np.isinf(m2):
        return None

    if np.isinf(m1):
        x = x1
        y = m2 * (x - x2) + y2
        return np.array([x, y], dtype=float)

    if np.isinf(m2):
        x = x2
        y = m1 * (x - x1) + y1
        return np.array([x, y], dtype=float)

    if abs(m1 - m2) < 1e-9:
        return None

    x = (m1 * x1 - m2 * x2 + y2 - y1) / (m1 - m2)
    y = y1 + m1 * (x - x1)
    return np.array([x, y], dtype=float)


def make_big_obstacle_grid(w=GRID_W, h=GRID_H):
    grid = np.zeros((h, w), dtype=np.uint8)

    n_obs = random.randint(4, 8)

    for _ in range(n_obs):
        ow = random.randint(3, max(4, int(w * 0.22)))
        oh = random.randint(3, max(4, int(h * 0.18)))

        x0 = random.randint(2, max(2, w - ow - 2))
        y0 = random.randint(2, max(2, h - oh - 2))

        grid[y0:y0 + oh, x0:x0 + ow] = 1

    return grid


def pick_start_goal(grid):
    h, w = grid.shape

    left_x = max(1, int(round(0.3 / CELL_SIZE_M)))
    right_x = min(w - 2, w - 1 - left_x)

    for _ in range(2000):
        sy = random.randint(2, h - 3)
        gy = random.randint(2, h - 3)

        start = (left_x, sy)
        goal = (right_x, gy)

        if grid[sy, left_x] == 0 and grid[gy, right_x] == 0:
            return start, goal

    return (left_x, h // 2), (right_x, h // 2)


def compute_arc_lengths(path):
    path = np.asarray(path, dtype=float)

    if len(path) == 0:
        return np.array([], dtype=float)

    s = [0.0]
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        s.append(s[-1] + np.sqrt(dx * dx + dy * dy))

    return np.array(s, dtype=float)


def find_lookahead_point(path, arc_len, robot_index, L_m):
    target_s = arc_len[robot_index] + L_m

    for i in range(robot_index, len(path)):
        if arc_len[i] >= target_s:
            return path[i], i

    return None, None


def plot_grid_image(plot, grid):
    img = pg.ImageItem()
    plot.addItem(img)

    display = np.where(grid > 0, 0, 255).astype(np.uint8)
    img.setImage(display.T)
    img.setRect(QtCore.QRectF(0, 0, MAP_W_M, MAP_H_M))

    plot.setXRange(0, MAP_W_M)
    plot.setYRange(0, MAP_H_M)
    plot.setAspectLocked(True)
    plot.showGrid(x=True, y=True, alpha=0.2)

    return img


def add_text(plot, x, y, text, color='k'):
    t = pg.TextItem(text=text, color=color, anchor=(0.5, 0.5))
    t.setPos(float(x), float(y))
    plot.addItem(t)
    return t


def get_forward_heading(path, idx, default_theta=0.0, look_pts=HEADING_LOOKAHEAD_PTS):
    if len(path) < 2:
        return angle_wrap(default_theta)

    idx = max(0, min(len(path) - 1, idx))
    ref_idx = min(idx + look_pts, len(path) - 1)

    p0 = np.asarray(path[idx], dtype=float)
    p1 = np.asarray(path[ref_idx], dtype=float)

    d = p1 - p0
    if np.linalg.norm(d) < 1e-9:
        for j in range(idx + 1, len(path)):
            p1 = np.asarray(path[j], dtype=float)
            d = p1 - p0
            if np.linalg.norm(d) >= 1e-9:
                return angle_wrap(np.arctan2(d[1], d[0]))
        return angle_wrap(default_theta)

    return angle_wrap(np.arctan2(d[1], d[0]))


def get_average_heading(path, idx, pts_ahead=SEGMENT_HEADING_LOOKAHEAD_PTS, default_theta=0.0):
    path = np.asarray(path, dtype=float)
    if len(path) < 2:
        return angle_wrap(default_theta)

    idx = max(0, min(len(path) - 2, idx))
    end_idx = min(len(path) - 1, idx + pts_ahead)

    headings = []
    for k in range(idx, end_idx):
        d = path[k + 1] - path[k]
        if np.linalg.norm(d) > 1e-9:
            headings.append(np.arctan2(d[1], d[0]))

    if not headings:
        return angle_wrap(default_theta)

    c = np.mean(np.cos(headings))
    s = np.mean(np.sin(headings))
    return angle_wrap(np.arctan2(s, c))


def world_delta_to_body(delta_world, heading):
    dx, dy = float(delta_world[0]), float(delta_world[1])
    c = np.cos(heading)
    s = np.sin(heading)

    body_x = c * dx + s * dy
    body_y = -s * dx + c * dy
    return body_x, body_y


# ============================================================
# BACKWARD PLANNER FROM REQUIRED FINAL HEADING
# ============================================================
def build_navigation_modes(modified_path, goal_final_theta_rad):
    """
    Plan backward from a required final heading.

    Final segment is forced to preserve the chosen goal heading.
    Earlier segments get new headings by inserting TIP points when the
    current heading becomes too hard to maintain.

    Modes:
      ACK  = tangent-follow
      CRAB = preserve-heading translation
      TIP  = rotate in place to heading of next segment
    """
    modified_path = np.asarray(modified_path, dtype=float)
    n = len(modified_path)

    if n == 0:
        return [], [], {}

    mode_by_index = ["ACK"] * n
    heading_ref_by_index = [float(goal_final_theta_rad)] * n
    tip_target_heading = {}

    if n == 1:
        return mode_by_index, heading_ref_by_index, tip_target_heading

    theta_ref = angle_wrap(goal_final_theta_rad)
    segment_is_final = True
    lateral_streak = 0

    i = n - 2
    while i >= 0:
        heading_ref_by_index[i] = float(theta_ref)

        path_theta = get_forward_heading(modified_path, i, default_theta=theta_ref)
        err = abs(angle_wrap(path_theta - theta_ref))

        delta = modified_path[i + 1] - modified_path[i]
        bx, by = world_delta_to_body(delta, theta_ref)

        if abs(by) > TIP_LATERAL_RATIO * max(abs(bx), 1e-9):
            lateral_streak += 1
        else:
            lateral_streak = 0

        # Force preserve-heading on the final segment so the robot truly ends at the chosen angle
        if segment_is_final:
            mode_by_index[i] = "CRAB"
        else:
            mode_by_index[i] = "ACK" if err <= ACK_MAX_ERR_RAD else "CRAB"

        # If this heading becomes too awkward for earlier path, insert a TIP at i+1
        # and choose a new heading for the previous segment.
        force_tip = (err >= TIP_TRIGGER_ERR_RAD) or (lateral_streak >= TIP_LATERAL_STREAK)

        if force_tip and i < n - 2:
            tip_idx = i + 1
            mode_by_index[tip_idx] = "TIP"
            tip_target_heading[tip_idx] = float(theta_ref)

            # choose new heading for the earlier segment
            new_theta = get_average_heading(
                modified_path,
                max(0, i - SEGMENT_HEADING_LOOKAHEAD_PTS + 1),
                pts_ahead=SEGMENT_HEADING_LOOKAHEAD_PTS,
                default_theta=path_theta
            )

            theta_ref = float(new_theta)
            segment_is_final = False
            lateral_streak = 0

            # reprocess this same i under the new earlier-segment heading
            continue

        i -= 1

    heading_ref_by_index[-1] = float(goal_final_theta_rad)
    if n >= 2:
        mode_by_index[-1] = mode_by_index[-2]

    return mode_by_index, heading_ref_by_index, tip_target_heading


# ============================================================
# SECTION 1:
# Create random obstacle map + A* + Bezier smoothing
# OUTPUT PATHS IN METERS
# ============================================================
def build_section1_environment_and_bezier():
    grid = make_big_obstacle_grid()
    start_grid, goal_grid = pick_start_goal(grid)

    inflated = inflate_obstacles(grid, ROBOT_RADIUS_M, CELL_SIZE_M)
    path_grid = astar(inflated, start_grid, goal_grid)

    if path_grid is None or len(path_grid) < 2:
        return None

    path_grid_float = [(float(x), float(y)) for (x, y) in path_grid]

    if len(path_grid_float) >= 3:
        smooth_path_grid = np.asarray(bezier(path_grid_float, n=max(80, len(path_grid_float) * 4)), dtype=float)
    else:
        smooth_path_grid = np.asarray(path_grid_float, dtype=float)

    path_m = np.asarray([grid_to_world_point(p) for p in path_grid_float], dtype=float)
    smooth_path_m = np.asarray([grid_to_world_point(p) for p in smooth_path_grid], dtype=float)

    start_m = grid_to_world_point(start_grid)
    goal_m = grid_to_world_point(goal_grid)

    return {
        "grid": grid,
        "start": start_m,
        "goal": goal_m,
        "path": path_m,
        "smooth_path": smooth_path_m,
    }


# ============================================================
# SECTION 2:
# Input smooth path in METERS, output segmented path in METERS
# ============================================================
def build_section2_segmented_path(section1_data):
    grid = section1_data["grid"]
    start = np.asarray(section1_data["start"], dtype=float)
    goal = np.asarray(section1_data["goal"], dtype=float)
    path = np.asarray(section1_data["path"], dtype=float)
    smooth_path = np.asarray(section1_data["smooth_path"], dtype=float)

    arc_len = compute_arc_lengths(smooth_path)

    robot_positions = []
    lookahead_points = []
    R_values = []

    for i in range(len(smooth_path) - 2):
        robot_pos = smooth_path[i]
        lookahead, _ = find_lookahead_point(smooth_path, arc_len, i, LOOKAHEAD_DISTANCE_M)

        if lookahead is None:
            break

        rx, ry = robot_pos
        lx, ly = lookahead

        dx = smooth_path[i + 1][0] - smooth_path[i][0]
        dy = smooth_path[i + 1][1] - smooth_path[i][1]
        theta = np.arctan2(dy, dx)

        dxr = lx - rx
        dyr = ly - ry

        delta_y = -np.sin(theta) * dxr + np.cos(theta) * dyr

        if abs(delta_y) > 1e-9:
            R = (LOOKAHEAD_DISTANCE_M ** 2) / (2.0 * delta_y)
        else:
            R = None

        robot_positions.append(np.array(robot_pos, dtype=float))
        lookahead_points.append(np.array(lookahead, dtype=float))
        R_values.append(R)

    turning_points = {}
    turn_index = 0
    in_turn = False

    for k in range(len(robot_positions)):
        robot_pos = robot_positions[k]
        lookahead = lookahead_points[k]
        R = R_values[k]

        tight_turn = (R is not None and abs(R) < R_MIN_M)

        dx = smooth_path[k + 1][0] - smooth_path[k][0]
        dy = smooth_path[k + 1][1] - smooth_path[k][1]
        slope = dy / dx if abs(dx) > 1e-9 else np.inf

        if tight_turn and not in_turn:
            turning_points[turn_index] = {
                "start": {
                    "point": np.array(robot_pos, dtype=float),
                    "slope": slope,
                    "R": R,
                    "intersection": np.array(lookahead, dtype=float)
                }
            }
            in_turn = True

        elif (not tight_turn) and in_turn:
            turning_points[turn_index]["end"] = {
                "point": np.array(robot_pos, dtype=float),
                "slope": slope,
                "intersection": np.array(lookahead, dtype=float)
            }
            turn_index += 1
            in_turn = False

    if in_turn and len(robot_positions) > 0:
        k = len(robot_positions) - 1
        dx = smooth_path[k + 1][0] - smooth_path[k][0]
        dy = smooth_path[k + 1][1] - smooth_path[k][1]
        slope = dy / dx if abs(dx) > 1e-9 else np.inf

        turning_points[turn_index]["end"] = {
            "point": np.array(robot_positions[k], dtype=float),
            "slope": slope,
            "intersection": np.array(lookahead_points[k], dtype=float)
        }

    turn_intersections = []

    for tp in turning_points.values():
        if "end" not in tp:
            continue

        start_pt = tp["start"]["point"]
        start_slope = tp["start"]["slope"]
        end_pt = tp["end"]["point"]
        end_slope = tp["end"]["slope"]

        intersection = slope_intersection(start_pt, start_slope, end_pt, end_slope)
        if intersection is not None:
            turn_intersections.append(np.array(intersection, dtype=float))

    modified_path = []
    turn_list = list(turning_points.values())

    i = 0
    while i < len(smooth_path):
        p = tuple(smooth_path[i])
        replaced = False

        for tp in turn_list:
            if "end" not in tp:
                continue

            start_pt = np.array(tp["start"]["point"], dtype=float)
            end_pt = np.array(tp["end"]["point"], dtype=float)
            start_slope = tp["start"]["slope"]
            end_slope = tp["end"]["slope"]

            if np.linalg.norm(np.array(p, dtype=float) - start_pt) < POINT_MATCH_TOL_M:
                intersection = slope_intersection(start_pt, start_slope, end_pt, end_slope)

                modified_path.append(start_pt)

                use_intersection = False
                if intersection is not None:
                    v1 = intersection - start_pt
                    v2 = end_pt - intersection
                    v_direct = end_pt - start_pt

                    if (
                        np.linalg.norm(v1) > 1e-6
                        and np.linalg.norm(v2) > 1e-6
                        and np.linalg.norm(v_direct) > 1e-6
                    ):
                        c1 = np.dot(v1, v_direct)
                        c2 = np.dot(v2, v_direct)

                        if c1 > 0 and c2 > 0:
                            use_intersection = True

                if use_intersection:
                    modified_path.append(np.array(intersection, dtype=float))

                modified_path.append(end_pt)

                while i < len(smooth_path):
                    if np.linalg.norm(smooth_path[i] - end_pt) < POINT_MATCH_TOL_M:
                        break
                    i += 1

                replaced = True
                break

        if not replaced:
            modified_path.append(np.array(p, dtype=float))

        i += 1

    modified_path = np.array(modified_path, dtype=float)

    clean_path = [modified_path[0]]
    for p in modified_path[1:]:
        if np.linalg.norm(p - clean_path[-1]) > 1e-9:
            clean_path.append(p)

    modified_path = np.array(clean_path, dtype=float)
    modified_path = densify_path(modified_path, step=DENSIFY_STEP_M)

    goal_final_theta_rad = np.radians(GOAL_FINAL_THETA_DEG)

    mode_by_index, heading_ref_by_index, tip_target_heading = build_navigation_modes(
        modified_path,
        goal_final_theta_rad
    )

    tip_indices = [idx for idx, mode in enumerate(mode_by_index) if mode == "TIP"]

    return {
        "grid": grid,
        "start": start,
        "goal": goal,
        "path": path,
        "smooth_path": smooth_path,
        "robot_positions": robot_positions,
        "lookahead_points": lookahead_points,
        "R_values": R_values,
        "turning_points": turning_points,
        "turn_intersections": turn_intersections,
        "modified_path": modified_path,
        "tip_indices": tip_indices,
        "mode_by_index": mode_by_index,
        "heading_ref_by_index": heading_ref_by_index,
        "tip_target_heading": tip_target_heading,
        "goal_final_theta_rad": goal_final_theta_rad,
    }


def build_section2_segmented_path_from_smooth_path(
    smooth_path_m,
    raw_path_m=None,
    grid=None,
    start_m=None,
    goal_m=None
):
    smooth_path_m = np.asarray(smooth_path_m, dtype=float)

    if raw_path_m is None:
        raw_path_m = np.asarray(smooth_path_m, dtype=float)
    else:
        raw_path_m = np.asarray(raw_path_m, dtype=float)

    if start_m is None:
        start_m = np.asarray(smooth_path_m[0], dtype=float)
    else:
        start_m = np.asarray(start_m, dtype=float)

    if goal_m is None:
        goal_m = np.asarray(smooth_path_m[-1], dtype=float)
    else:
        goal_m = np.asarray(goal_m, dtype=float)

    section1_like = {
        "grid": grid,
        "start": start_m,
        "goal": goal_m,
        "path": raw_path_m,
        "smooth_path": smooth_path_m,
    }
    return build_section2_segmented_path(section1_like)


def build_sim_data():
    section1 = build_section1_environment_and_bezier()
    if section1 is None:
        return None
    return build_section2_segmented_path(section1)


class SimulationWindow:
    def __init__(self, data):
        self.data = data
        self.grid = data["grid"]
        self.start = data["start"]
        self.goal = data["goal"]
        self.path = data["path"]
        self.smooth_path = data["smooth_path"]
        self.turning_points = data["turning_points"]
        self.turn_intersections = data["turn_intersections"]
        self.modified_path = data["modified_path"]
        self.tip_indices = data["tip_indices"]
        self.mode_by_index = data["mode_by_index"]
        self.heading_ref_by_index = data["heading_ref_by_index"]
        self.tip_target_heading = data["tip_target_heading"]
        self.goal_final_theta_rad = data["goal_final_theta_rad"]

        self.app = QApplication.instance() or QApplication([])

        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")

        self.win = pg.GraphicsLayoutWidget(title="Local Motion Simulation (Metric)")
        self.win.resize(1800, 1000)

        self.sim_timer = None
        self.graph_timer = None

        self.setup_plots()
        self.initialize_sim_state()
        self.setup_timers()

    def restart_simulation(self):
        if self.sim_timer is not None:
            self.sim_timer.stop()
        if self.graph_timer is not None:
            self.graph_timer.stop()

        self.win.clear()

        data = None
        for _ in range(20):
            section1 = build_section1_environment_and_bezier()
            if section1 is None:
                continue

            data = build_section2_segmented_path(section1)
            if data is not None:
                break

        if data is None:
            print("Failed to generate new path")
            return

        self.data = data
        self.grid = data["grid"]
        self.start = data["start"]
        self.goal = data["goal"]
        self.path = data["path"]
        self.smooth_path = data["smooth_path"]
        self.turning_points = data["turning_points"]
        self.turn_intersections = data["turn_intersections"]
        self.modified_path = data["modified_path"]
        self.tip_indices = data["tip_indices"]
        self.mode_by_index = data["mode_by_index"]
        self.heading_ref_by_index = data["heading_ref_by_index"]
        self.tip_target_heading = data["tip_target_heading"]
        self.goal_final_theta_rad = data["goal_final_theta_rad"]

        self.setup_plots()
        self.initialize_sim_state()
        self.setup_timers()

    def draw_heading_arrow(self, plot, p, theta, length=0.20, pen=None):
        if pen is None:
            pen = pg.mkPen((80, 80, 80), width=2)
        x2 = p[0] + length * np.cos(theta)
        y2 = p[1] + length * np.sin(theta)
        plot.plot([p[0], x2], [p[1], y2], pen=pen)

    def draw_segmented_mode_path(self, plot):
        colors = {
            "ACK": pg.mkPen((0, 140, 0), width=3),
            "CRAB": pg.mkPen((255, 165, 0), width=3),
            "TIP": pg.mkPen((0, 0, 180), width=3),
        }

        for i in range(len(self.modified_path) - 1):
            p0 = self.modified_path[i]
            p1 = self.modified_path[i + 1]
            mode = self.mode_by_index[i]
            pen = colors.get(mode, colors["ACK"])
            plot.plot([p0[0], p1[0]], [p0[1], p1[1]], pen=pen)

    def setup_plots(self):
        # 1) MAP
        self.plot_map = self.win.addPlot(title="Random Obstacles + A* + Bezier (meters)")
        if self.grid is not None:
            plot_grid_image(self.plot_map, self.grid)
        else:
            self.plot_map.setXRange(0, MAP_W_M)
            self.plot_map.setYRange(0, MAP_H_M)
            self.plot_map.setAspectLocked(True)
            self.plot_map.showGrid(x=True, y=True, alpha=0.2)

        self.plot_map.plot(self.path[:, 0], self.path[:, 1], pen=pg.mkPen('y', width=2))
        self.plot_map.plot(self.smooth_path[:, 0], self.smooth_path[:, 1], pen=pg.mkPen('r', width=3))
        self.plot_map.plot([self.start[0]], [self.start[1]], pen=None, symbol='o', symbolBrush='g', symbolSize=12)
        self.plot_map.plot([self.goal[0]], [self.goal[1]], pen=None, symbol='o', symbolBrush='r', symbolSize=12)

        # 2) PATH ANALYSIS
        self.plot_path = self.win.addPlot(title="Path Analysis")
        self.plot_path.setXRange(0, MAP_W_M)
        self.plot_path.setYRange(0, MAP_H_M)
        self.plot_path.setAspectLocked(True)
        self.plot_path.showGrid(x=True, y=True, alpha=0.2)
        self.plot_path.plot(self.smooth_path[:, 0], self.smooth_path[:, 1], pen=pg.mkPen('r', width=3))

        for tp in self.turning_points.values():
            start_point = tp["start"]["point"]
            start_R = tp["start"]["R"]
            intersection = tp["start"]["intersection"]

            self.plot_path.plot([start_point[0]], [start_point[1]], pen=None, symbol='o', symbolBrush='b', symbolSize=10)
            self.plot_path.plot([intersection[0]], [intersection[1]], pen=None, symbol='o', symbolBrush=(255, 165, 0), symbolSize=10)

            circle = QtCore.QRectF(
                start_point[0] - LOOKAHEAD_DISTANCE_M,
                start_point[1] - LOOKAHEAD_DISTANCE_M,
                2 * LOOKAHEAD_DISTANCE_M,
                2 * LOOKAHEAD_DISTANCE_M
            )
            c = pg.QtWidgets.QGraphicsEllipseItem(circle)
            c.setPen(pg.mkPen('g', width=2))
            self.plot_path.addItem(c)

            add_text(self.plot_path, start_point[0] + 0.12, start_point[1] + 0.12, f"R={start_R:.2f}m")

            if "end" in tp:
                end_point = tp["end"]["point"]
                self.plot_path.plot([end_point[0]], [end_point[1]], pen=None, symbol='o', symbolBrush=(128, 0, 128), symbolSize=10)

        # 3) TURN CALC
        self.win.nextRow()
        self.plot_turn = self.win.addPlot(title="Turning Point Calculation")
        self.plot_turn.setXRange(0, MAP_W_M)
        self.plot_turn.setYRange(0, MAP_H_M)
        self.plot_turn.setAspectLocked(True)
        self.plot_turn.showGrid(x=True, y=True, alpha=0.2)
        self.plot_turn.plot(self.smooth_path[:, 0], self.smooth_path[:, 1], pen=pg.mkPen('r', width=3))

        color_list = [
            (31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40),
            (148, 103, 189), (140, 86, 75), (227, 119, 194), (127, 127, 127)
        ]

        for idx, (_, tp) in enumerate(self.turning_points.items()):
            color = color_list[idx % len(color_list)]
            pen = pg.mkPen(color, width=2)

            start_pt = tp["start"]["point"]
            start_slope = tp["start"]["slope"]
            x0, y0 = start_pt
            self.plot_turn.plot([x0], [y0], pen=None, symbol='o', symbolBrush=color, symbolSize=10)

            if np.isinf(start_slope):
                self.plot_turn.plot([x0, x0], [y0, y0 + SLOPE_LENGTH_M], pen=pen)
            else:
                dx = SLOPE_LENGTH_M
                dy = start_slope * dx
                self.plot_turn.plot([x0, x0 + dx], [y0, y0 + dy], pen=pen)

            if "end" in tp:
                end_pt = tp["end"]["point"]
                end_slope = tp["end"]["slope"]
                x1, y1 = end_pt
                self.plot_turn.plot([x1], [y1], pen=None, symbol='o', symbolBrush=color, symbolSize=10)

                if np.isinf(end_slope):
                    self.plot_turn.plot([x1, x1], [y1, y1 - SLOPE_LENGTH_M], pen=pen)
                else:
                    dx = -SLOPE_LENGTH_M
                    dy = end_slope * dx
                    self.plot_turn.plot([x1, x1 + dx], [y1, y1 + dy], pen=pen)

                intersection = slope_intersection(start_pt, start_slope, end_pt, end_slope)
                if intersection is not None:
                    ix, iy = intersection
                    self.plot_turn.plot([ix], [iy], pen=None, symbol='o', symbolBrush='y', symbolSize=9)

        # 4) MODIFIED PATH
        self.plot_modified = self.win.addPlot(title="New Path With Turning Point")
        self.plot_modified.setXRange(0, MAP_W_M)
        self.plot_modified.setYRange(0, MAP_H_M)
        self.plot_modified.setAspectLocked(True)
        self.plot_modified.showGrid(x=True, y=True, alpha=0.2)
        self.plot_modified.plot(self.modified_path[:, 0], self.modified_path[:, 1], pen=pg.mkPen('r', width=3))

        for ti in self.tip_indices:
            p = self.modified_path[ti]
            self.plot_modified.plot([p[0]], [p[1]], pen=None, symbol='o', symbolBrush='y', symbolSize=10)

        # 5) MODE DECIDER
        self.win.nextRow()
        self.plot_mode = self.win.addPlot(title="Navigation Segmentation (ACK / PRESERVE / TIP)")
        self.plot_mode.setXRange(0, MAP_W_M)
        self.plot_mode.setYRange(0, MAP_H_M)
        self.plot_mode.setAspectLocked(True)
        self.plot_mode.showGrid(x=True, y=True, alpha=0.2)

        self.draw_segmented_mode_path(self.plot_mode)

        self.plot_mode.plot([self.start[0]], [self.start[1]], pen=None, symbol='o', symbolBrush='g', symbolSize=12)
        self.plot_mode.plot([self.goal[0]], [self.goal[1]], pen=None, symbol='o', symbolBrush='r', symbolSize=12)

        run_start = 0
        for i in range(1, len(self.mode_by_index)):
            if self.mode_by_index[i] != self.mode_by_index[run_start]:
                mid = (run_start + i - 1) // 2
                pm = self.modified_path[mid]
                label = self.mode_by_index[run_start]
                if label == "ACK":
                    add_text(self.plot_mode, pm[0], pm[1], "ACK", color='darkgreen')
                elif label == "CRAB":
                    add_text(self.plot_mode, pm[0], pm[1], "PRESERVE", color=(255, 140, 0))
                else:
                    add_text(self.plot_mode, pm[0], pm[1], "TIP", color=(0, 0, 180))
                run_start = i

        if len(self.mode_by_index) > 0:
            mid = (run_start + len(self.mode_by_index) - 1) // 2
            pm = self.modified_path[mid]
            label = self.mode_by_index[run_start]
            if label == "ACK":
                add_text(self.plot_mode, pm[0], pm[1], "ACK", color='darkgreen')
            elif label == "CRAB":
                add_text(self.plot_mode, pm[0], pm[1], "PRESERVE", color=(255, 140, 0))
            else:
                add_text(self.plot_mode, pm[0], pm[1], "TIP", color=(0, 0, 180))

        for k, ti in enumerate(self.tip_indices, start=1):
            p = self.modified_path[ti]
            self.plot_mode.plot([p[0]], [p[1]], pen=None, symbol='o', symbolBrush=(0, 0, 180), symbolSize=12)
            add_text(self.plot_mode, p[0], p[1] + 0.12, f"TIP{k}", color=(0, 0, 180))

        step = max(1, len(self.modified_path) // 18)
        arrow_len = 0.18
        for i in range(0, len(self.modified_path), step):
            p = self.modified_path[i]
            th = self.heading_ref_by_index[i]
            x2 = p[0] + arrow_len * np.cos(th)
            y2 = p[1] + arrow_len * np.sin(th)
            self.plot_mode.plot([p[0], x2], [p[1], y2], pen=pg.mkPen((120, 120, 120), width=1))

        # final goal orientation arrow
        self.draw_heading_arrow(
            self.plot_mode,
            self.goal,
            self.goal_final_theta_rad,
            length=0.28,
            pen=pg.mkPen((180, 0, 180), width=3)
        )
        add_text(
            self.plot_mode,
            self.goal[0] + 0.28 * np.cos(self.goal_final_theta_rad),
            self.goal[1] + 0.28 * np.sin(self.goal_final_theta_rad) + 0.10,
            f"GOAL θ={GOAL_FINAL_THETA_DEG:.0f}°",
            color=(180, 0, 180)
        )

        # 6) ANIMATION
        self.plot_anim = self.win.addPlot(title="Robot Motion Animation")
        self.plot_anim.setXRange(0, MAP_W_M)
        self.plot_anim.setYRange(0, MAP_H_M)
        self.plot_anim.setAspectLocked(True)
        self.plot_anim.showGrid(x=True, y=True, alpha=0.2)
        self.draw_segmented_mode_path(self.plot_anim)

        self.draw_heading_arrow(
            self.plot_anim,
            self.goal,
            self.goal_final_theta_rad,
            length=0.28,
            pen=pg.mkPen((180, 0, 180), width=3)
        )

        self.robot_dot = self.plot_anim.plot(
            [self.modified_path[0, 0]],
            [self.modified_path[0, 1]],
            pen=None,
            symbol='o',
            symbolBrush='b',
            symbolSize=12
        )

        self.heading_line = self.plot_anim.plot([], [], pen=pg.mkPen('y', width=3))
        self.info_text = pg.TextItem("", anchor=(0, 0))
        self.plot_anim.addItem(self.info_text)

    def initialize_sim_state(self):
        if len(self.modified_path) >= 1:
            self.theta = float(self.heading_ref_by_index[0])
        else:
            self.theta = 0.0

        self.prev_desired_theta = float(self.theta)

        self.vx_current = 0.0
        self.vy_current = 0.0
        self.vx_target = BOT_SPEED_MPS
        self.vy_target = BOT_SPEED_MPS
        self.accel = BOT_ACCEL

        self.tip_set = set(self.tip_indices)
        self.path_index = 0
        self.heading_len = 0.15

        self.mode = self.mode_by_index[0] if len(self.mode_by_index) > 0 else "ACK"
        self.turn_target_theta = None

        self.render_pos = np.array(self.modified_path[0], dtype=float)
        self.render_theta = float(self.theta)
        self.render_text = "Initializing..."
        self.sim_delay_ms = FRAME_DELAY_FAST_MS

    def setup_timers(self):
        if self.sim_timer is not None:
            self.sim_timer.stop()
        if self.graph_timer is not None:
            self.graph_timer.stop()

        self.sim_timer = QtCore.QTimer()
        self.sim_timer.timeout.connect(self.state_machine_step)
        self.sim_timer.start(self.sim_delay_ms)

        self.graph_timer = QtCore.QTimer()
        self.graph_timer.timeout.connect(self.update_graphics)
        self.graph_timer.start(GRAPH_DELAY_MS)

    def set_sim_timer_delay(self, ms):
        self.sim_delay_ms = int(ms)
        if self.sim_timer is not None:
            self.sim_timer.start(self.sim_delay_ms)

    def state_machine_step(self):
        if self.path_index >= len(self.modified_path) - 1:
            if self.sim_timer is not None:
                self.sim_timer.stop()
            if self.graph_timer is not None:
                self.graph_timer.stop()

            QtCore.QTimer.singleShot(1000, self.restart_simulation)
            return

        p = self.modified_path[self.path_index]
        next_p = self.modified_path[self.path_index + 1]
        delta = next_p - p

        # TIP MODE
        if self.mode == "TIP":
            dtheta = angle_wrap(self.turn_target_theta - self.theta)

            if abs(dtheta) < TIP_STOP_TOL:
                self.theta = angle_wrap(self.turn_target_theta)
                self.render_pos = np.array(p, dtype=float)
                self.render_theta = float(self.theta)
                self.render_text = (
                    f"TIP ROTATION COMPLETE\n"
                    f"theta={np.degrees(self.theta) % 360:.1f}°\n"
                    f"idx={self.path_index}"
                )

                self.path_index += 1
                if self.path_index < len(self.mode_by_index):
                    self.mode = self.mode_by_index[self.path_index]
                else:
                    self.mode = "ACK"

                self.vx_current = 0.0
                self.vy_current = 0.0
                self.set_sim_timer_delay(FRAME_DELAY_FAST_MS)
                return

            self.theta = angle_wrap(self.theta + TIP_TURN_RATE * dtheta)
            wheels_tip = mecanum_tip(dtheta)

            self.render_pos = np.array(p, dtype=float)
            self.render_theta = float(self.theta)

            theta_deg = np.degrees(self.theta) % 360
            self.render_text = (
                f"TIP ROTATION\n"
                f"w1={wheels_tip[0]:.2f}\n"
                f"w2={wheels_tip[1]:.2f}\n"
                f"w3={wheels_tip[2]:.2f}\n"
                f"w4={wheels_tip[3]:.2f}\n"
                f"theta={theta_deg:.1f}°\n"
                f"idx={self.path_index}"
            )
            return

        # TIP ENTRY
        if self.path_index in self.tip_set:
            self.turn_target_theta = self.tip_target_heading.get(
                self.path_index,
                self.heading_ref_by_index[self.path_index]
            )
            self.mode = "TIP"
            self.vx_current = 0.0
            self.vy_current = 0.0
            self.set_sim_timer_delay(TIP_DELAY_MS)
            return

        mode_here = self.mode_by_index[self.path_index]

        next_tips = [t for t in self.tip_indices if t >= self.path_index]
        remaining = (min(next_tips) - self.path_index) * DENSIFY_STEP_M if next_tips else 999999.0

        if remaining < TIP_STOP_RADIUS_M:
            self.set_sim_timer_delay(FRAME_DELAY_SLOW_MS)
        elif remaining < TIP_SLOW_RADIUS_M:
            self.set_sim_timer_delay(FRAME_DELAY_MED_MS)
        else:
            self.set_sim_timer_delay(FRAME_DELAY_FAST_MS)

        # ACK MODE
        if mode_here == "ACK":
            desired_theta = get_forward_heading(
                self.modified_path,
                self.path_index,
                default_theta=self.theta,
                look_pts=HEADING_LOOKAHEAD_PTS
            )

            jump = angle_wrap(desired_theta - self.prev_desired_theta)
            if abs(jump) > MAX_HEADING_JUMP_RAD:
                desired_theta = self.prev_desired_theta

            self.prev_desired_theta = float(desired_theta)

            omega_cmd = angle_wrap(desired_theta - self.theta)

            self.vx_current += self.accel * (self.vx_target - self.vx_current)
            vx_cmd = self.vx_current

            wheels = mecanum_ack(vx_cmd, omega_cmd)

            dtheta = angle_wrap(desired_theta - self.theta)
            turn_rate = 0.15 if self.path_index == 0 else 0.25
            self.theta = angle_wrap(self.theta + turn_rate * dtheta)

            self.vy_current += self.accel * (0.0 - self.vy_current)

            self.render_pos = np.array(p, dtype=float)
            self.render_theta = float(self.theta)

            theta_deg = np.degrees(self.theta) % 360
            self.render_text = (
                f"ACK MODE\n"
                f"w1={wheels[0]:.2f}\n"
                f"w2={wheels[1]:.2f}\n"
                f"w3={wheels[2]:.2f}\n"
                f"w4={wheels[3]:.2f}\n"
                f"theta={theta_deg:.1f}°\n"
                f"idx={self.path_index}"
            )

        # PRESERVE-HEADING MODE
        elif mode_here == "CRAB":
            desired_theta = self.heading_ref_by_index[self.path_index]
            omega_cmd = angle_wrap(desired_theta - self.theta)
            self.theta = angle_wrap(self.theta + PRESERVE_HEADING_GAIN * omega_cmd)

            bx, by = world_delta_to_body(delta, desired_theta)

            vx_dir = 0.0 if abs(bx) < MIN_TRANS_COMPONENT else np.sign(bx)
            vy_dir = 0.0 if abs(by) < MIN_TRANS_COMPONENT else np.sign(by)

            self.vx_current += self.accel * (vx_dir * self.vx_target - self.vx_current)
            self.vy_current += self.accel * (vy_dir * self.vy_target - self.vy_current)

            wheels = mecanum_preserve(self.vx_current, self.vy_current, omega_cmd)

            self.render_pos = np.array(p, dtype=float)
            self.render_theta = float(self.theta)

            theta_deg = np.degrees(self.theta) % 360
            self.render_text = (
                f"PRESERVE-HEADING MODE\n"
                f"body_x={bx:.3f} m\n"
                f"body_y={by:.3f} m\n"
                f"w1={wheels[0]:.2f}\n"
                f"w2={wheels[1]:.2f}\n"
                f"w3={wheels[2]:.2f}\n"
                f"w4={wheels[3]:.2f}\n"
                f"theta_ref={np.degrees(desired_theta) % 360:.1f}°\n"
                f"theta={theta_deg:.1f}°\n"
                f"idx={self.path_index}"
            )

        else:
            self.render_pos = np.array(p, dtype=float)
            self.render_theta = float(self.theta)
            self.render_text = f"UNKNOWN MODE @ idx={self.path_index}"

        self.mode = mode_here
        self.path_index += 1

    def update_graphics(self):
        p = self.render_pos
        theta = self.render_theta

        self.robot_dot.setData([p[0]], [p[1]])

        hx = p[0] + self.heading_len * np.cos(theta)
        hy = p[1] + self.heading_len * np.sin(theta)
        self.heading_line.setData([p[0], hx], [p[1], hy])

        self.info_text.setText(self.render_text)
        self.info_text.setPos(p[0] + 0.12, p[1] + 0.12)

    def run(self):
        self.win.show()
        self.app.exec_()


# ============================================================
# SECTION 3:
# Run sim + state machine
# ============================================================
def run_section3_sim_and_state_machine(data):
    print("Analysis points:", len(data["robot_positions"]))
    print(f"\nRequired final heading: {GOAL_FINAL_THETA_DEG:.1f} deg")

    print("\nTurning Points:\n")
    for i, tp in data["turning_points"].items():
        print(f"TurningPoint[{i}]")
        print("  start:")
        print(f"    point = {tp['start']['point']}")
        print(f"    slope = {tp['start']['slope']}")
        print(f"    R = {tp['start']['R']}")
        if "end" in tp:
            print("  end:")
            print(f"    point = {tp['end']['point']}")
            print(f"    slope = {tp['end']['slope']}")

    print("\nMode Sequence:\n")
    last = None
    for i, mode in enumerate(data["mode_by_index"]):
        if mode != last:
            if mode == "TIP":
                tgt = data["tip_target_heading"].get(i, 0.0)
                print(f"idx {i:4d}: TIP -> target heading {np.degrees(tgt):.1f} deg")
            else:
                href = data["heading_ref_by_index"][i]
                print(f"idx {i:4d}: {mode} @ heading_ref {np.degrees(href):.1f} deg")
            last = mode

    ack_count = sum(1 for m in data["mode_by_index"] if m == "ACK")
    crab_count = sum(1 for m in data["mode_by_index"] if m == "CRAB")
    tip_count = sum(1 for m in data["mode_by_index"] if m == "TIP")
    print(f"\nMode counts: ACK={ack_count}, PRESERVE={crab_count}, TIP={tip_count}")

    span = data["smooth_path"].max(axis=0) - data["smooth_path"].min(axis=0)
    print(f"\nPath span (m): x={span[0]:.3f}, y={span[1]:.3f}")
    print(f"Lookahead (m): {LOOKAHEAD_DISTANCE_M}")
    print(f"R_min (m): {R_MIN_M}")

    sim = SimulationWindow(data)
    sim.run()


def main():
    data = None

    for _ in range(20):
        section1 = build_section1_environment_and_bezier()
        if section1 is None:
            continue

        data = build_section2_segmented_path(section1)
        if data is not None:
            break

    if data is None:
        print("No path found after several tries")
        return

    run_section3_sim_and_state_machine(data)


if __name__ == "__main__":
    main()