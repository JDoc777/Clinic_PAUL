import random
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
from pyqtgraph.Qt import QtCore

from pathfinding import astar, inflate_obstacles, bezier


# =========================
# SIM CONFIG
# =========================
GRID_W = 140
GRID_H = 100
CELL_SIZE_M = 0.10
ROBOT_RADIUS_M = 0.25
SLOPE_LENGTH = 20

# =========================
# MOTION PARAMETERS
# =========================
BOT_SPEED = 0.1
BOT_ACCEL = 0.04
BOT_DECEL = 0.08

TIP_TURN_RATE = 0.06
TIP_STOP_TOL = np.radians(1)

LOOKAHEAD_DISTANCE_M = 1.2
LOOKAHEAD_DISTANCE = LOOKAHEAD_DISTANCE_M / CELL_SIZE_M

TIP_SLOW_RADIUS = 20
TIP_STOP_RADIUS = 5

FRAME_DELAY_FAST_MS = 20
FRAME_DELAY_MED_MS = 50
FRAME_DELAY_SLOW_MS = 100
TIP_DELAY_MS = 20

L = LOOKAHEAD_DISTANCE
R_min = 30


# =========================
# HELPERS
# =========================
def angle_wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def densify_path(path, step=0.5):
    dense = [path[0]]

    for i in range(len(path) - 1):
        p0 = np.array(path[i], dtype=float)
        p1 = np.array(path[i + 1], dtype=float)

        d = np.linalg.norm(p1 - p0)
        if d < 1e-9:
            continue

        n = max(1, int(np.ceil(d / step)))

        for k in range(1, n + 1):
            t = k / n
            p = p0 + t * (p1 - p0)
            dense.append(p)

    return np.array(dense)


def mecanum_ack(vx, omega, r=0.04, Lx=0.15, Ly=0.15):
    k = Lx + Ly

    M = np.array([
        [1, 0, -k],
        [1, 0,  k],
        [1, 0, -k],
        [1, 0,  k]
    ])

    V = np.array([vx, 0, omega])
    wheels = (1 / r) * (M @ V)

    MAX_WHEEL_SPEED = 31.42
    wheels = np.clip(wheels, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    return wheels


def mecanum_tip(omega, r=0.04, Lx=0.15, Ly=0.15):
    k = Lx + Ly

    M = np.array([
        [0, 0, -k],
        [0, 0,  k],
        [0, 0, -k],
        [0, 0,  k]
    ])

    V = np.array([0, 0, omega])
    wheels = (1 / r) * (M @ V)
    return wheels


def slope_intersection(p1, m1, p2, m2):
    x1, y1 = p1
    x2, y2 = p2

    if np.isinf(m1) and np.isinf(m2):
        return None

    if np.isinf(m1):
        x = x1
        y = m2 * (x - x2) + y2
        return x, y

    if np.isinf(m2):
        x = x2
        y = m1 * (x - x1) + y1
        return x, y

    if abs(m1 - m2) < 1e-6:
        return None

    x = (m1 * x1 - m2 * x2 + y2 - y1) / (m1 - m2)
    y = y1 + m1 * (x - x1)
    return x, y


def make_big_obstacle_grid(w=GRID_W, h=GRID_H):
    grid = np.zeros((h, w), dtype=np.uint8)

    n_obs = random.randint(4, 10)

    for _ in range(n_obs):
        ow = random.randint(6, 35)
        oh = random.randint(6, 30)

        x0 = random.randint(2, w - ow - 2)
        y0 = random.randint(2, h - oh - 2)

        grid[y0:y0 + oh, x0:x0 + ow] = 1

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


def compute_arc_lengths(path):
    s = [0.0]

    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        s.append(s[-1] + np.sqrt(dx * dx + dy * dy))

    return np.array(s)


def find_lookahead_point(path, arc_len, robot_index, L):
    target_s = arc_len[robot_index] + L

    for i in range(robot_index, len(path)):
        if arc_len[i] >= target_s:
            return path[i], i

    return None, None


def plot_grid_image(plot, grid):
    img = pg.ImageItem()
    plot.addItem(img)

    # obstacle=black, free=white
    display = np.where(grid > 0, 0, 255).astype(np.uint8)
    img.setImage(display.T)
    img.setRect(QtCore.QRectF(0, 0, grid.shape[1], grid.shape[0]))

    plot.setXRange(0, grid.shape[1])
    plot.setYRange(0, grid.shape[0])
    plot.setAspectLocked(True)
    plot.showGrid(x=True, y=True, alpha=0.2)

    return img


def add_text(plot, x, y, text, color='k'):
    t = pg.TextItem(text=text, color=color, anchor=(0.5, 0.5))
    t.setPos(float(x), float(y))
    plot.addItem(t)
    return t


def build_sim_data():
    grid = make_big_obstacle_grid()
    start, goal = pick_start_goal(grid)

    inflated = inflate_obstacles(grid, ROBOT_RADIUS_M, CELL_SIZE_M)
    path_grid = astar(inflated, start, goal)

    if path_grid is None or len(path_grid) < 2:
        return None

    path = [(float(x), float(y)) for (x, y) in path_grid]

    if len(path) >= 3:
        smooth_path = np.asarray(bezier(path, n=max(80, len(path) * 4)))
    else:
        smooth_path = np.asarray(path)

    arc_len = compute_arc_lengths(smooth_path)

    robot_positions = []
    lookahead_points = []
    R_values = []

    for i in range(len(smooth_path) - 2):
        robot_pos = smooth_path[i]
        lookahead, idx = find_lookahead_point(smooth_path, arc_len, i, L)

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

        if abs(delta_y) > 1e-6:
            R = (L ** 2) / (2 * delta_y)
        else:
            R = None

        robot_positions.append(robot_pos)
        lookahead_points.append(lookahead)
        R_values.append(R)

    turning_points = {}
    turn_index = 0
    in_turn = False

    for k in range(len(robot_positions)):
        robot_pos = robot_positions[k]
        lookahead = lookahead_points[k]
        R = R_values[k]

        tight_turn = (R is not None and abs(R) < R_min)

        dx = smooth_path[k + 1][0] - smooth_path[k][0]
        dy = smooth_path[k + 1][1] - smooth_path[k][1]
        slope = dy / dx if abs(dx) > 1e-6 else np.inf

        if tight_turn and not in_turn:
            turning_points[turn_index] = {
                "start": {
                    "point": robot_pos,
                    "slope": slope,
                    "R": R,
                    "intersection": lookahead
                }
            }
            in_turn = True

        elif not tight_turn and in_turn:
            turning_points[turn_index]["end"] = {
                "point": robot_pos,
                "slope": slope,
                "intersection": lookahead
            }
            turn_index += 1
            in_turn = False

    if in_turn and len(robot_positions) > 0:
        k = len(robot_positions) - 1
        dx = smooth_path[k + 1][0] - smooth_path[k][0]
        dy = smooth_path[k + 1][1] - smooth_path[k][1]
        slope = dy / dx if abs(dx) > 1e-6 else np.inf

        turning_points[turn_index]["end"] = {
            "point": robot_positions[k],
            "slope": slope,
            "intersection": lookahead_points[k]
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
            turn_intersections.append(np.array(intersection))

    modified_path = []
    turn_list = list(turning_points.values())

    i = 0
    while i < len(smooth_path):
        p = tuple(smooth_path[i])
        replaced = False

        for tp in turn_list:
            if "end" not in tp:
                continue

            start_pt = tuple(tp["start"]["point"])
            end_pt = tuple(tp["end"]["point"])
            start_slope = tp["start"]["slope"]
            end_slope = tp["end"]["slope"]

            if np.linalg.norm(np.array(p) - np.array(start_pt)) < 0.5:
                intersection = slope_intersection(start_pt, start_slope, end_pt, end_slope)

                modified_path.append(start_pt)
                if intersection is not None:
                    modified_path.append(intersection)
                modified_path.append(end_pt)

                while i < len(smooth_path):
                    if np.linalg.norm(smooth_path[i] - np.array(end_pt)) < 0.5:
                        break
                    i += 1

                replaced = True
                break

        if not replaced:
            modified_path.append(p)

        i += 1

    modified_path = np.array(modified_path)

    clean_path = [modified_path[0]]
    for p in modified_path[1:]:
        if np.linalg.norm(p - clean_path[-1]) > 1e-6:
            clean_path.append(p)

    modified_path = np.array(clean_path)
    modified_path = densify_path(modified_path, step=0.4)

    tip_indices = []
    for inter in turn_intersections:
        d = np.linalg.norm(modified_path - inter, axis=1)
        idx = int(np.argmin(d))
        tip_indices.append(idx)

    tip_indices = sorted(set(tip_indices))
    if 0 not in tip_indices:
        tip_indices.insert(0, 0)

    return {
        "grid": grid,
        "start": start,
        "goal": goal,
        "path": np.asarray(path),
        "smooth_path": smooth_path,
        "robot_positions": robot_positions,
        "lookahead_points": lookahead_points,
        "R_values": R_values,
        "turning_points": turning_points,
        "turn_intersections": turn_intersections,
        "modified_path": modified_path,
        "tip_indices": tip_indices,
    }


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

        self.app = QApplication.instance() or QApplication([])

        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")

        self.win = pg.GraphicsLayoutWidget(title="Local Motion Simulation")
        self.win.resize(1800, 1000)

        self.setup_plots()
        self.setup_animation()
    
    def restart_simulation(self):

        # clear all plots
        self.win.clear()

        # build new simulation data
        data = None
        for _ in range(20):
            data = build_sim_data()
            if data is not None:
                break

        if data is None:
            print("Failed to generate new path")
            return

        # overwrite state
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

        # rebuild plots
        self.setup_plots()

        # restart animation
        self.setup_animation()

    def setup_plots(self):
        # 1) MAP
        self.plot_map = self.win.addPlot(title="Random Obstacles + A* + Bezier")
        plot_grid_image(self.plot_map, self.grid)
        self.plot_map.plot(self.path[:, 0], self.path[:, 1], pen=pg.mkPen('y', width=2))
        self.plot_map.plot(self.smooth_path[:, 0], self.smooth_path[:, 1], pen=pg.mkPen('r', width=3))
        self.plot_map.plot([self.start[0]], [self.start[1]], pen=None, symbol='o', symbolBrush='g', symbolSize=12)
        self.plot_map.plot([self.goal[0]], [self.goal[1]], pen=None, symbol='o', symbolBrush='r', symbolSize=12)

        # 2) PATH ANALYSIS
        self.plot_path = self.win.addPlot(title="Path Analysis")
        self.plot_path.setXRange(0, GRID_W)
        self.plot_path.setYRange(0, GRID_H)
        self.plot_path.setAspectLocked(True)
        self.plot_path.showGrid(x=True, y=True, alpha=0.2)
        self.plot_path.plot(self.smooth_path[:, 0], self.smooth_path[:, 1], pen=pg.mkPen('r', width=3))

        for tp in self.turning_points.values():
            start_point = tp["start"]["point"]
            start_R = tp["start"]["R"]
            intersection = tp["start"]["intersection"]

            self.plot_path.plot([start_point[0]], [start_point[1]], pen=None, symbol='o', symbolBrush='b', symbolSize=10)
            self.plot_path.plot([intersection[0]], [intersection[1]], pen=None, symbol='o', symbolBrush=(255, 165, 0), symbolSize=10)

            circle = QtCore.QRectF(start_point[0] - L, start_point[1] - L, 2 * L, 2 * L)
            c = pg.QtWidgets.QGraphicsEllipseItem(circle)
            c.setPen(pg.mkPen('g', width=2))
            self.plot_path.addItem(c)

            add_text(self.plot_path, start_point[0] + 3, start_point[1] + 3, f"R={start_R:.1f}")

            if "end" in tp:
                end_point = tp["end"]["point"]
                self.plot_path.plot([end_point[0]], [end_point[1]], pen=None, symbol='o', symbolBrush=(128, 0, 128), symbolSize=10)

        # 3) TURN CALC
        self.win.nextRow()
        self.plot_turn = self.win.addPlot(title="Turning Point Calculation")
        self.plot_turn.setXRange(0, GRID_W)
        self.plot_turn.setYRange(0, GRID_H)
        self.plot_turn.setAspectLocked(True)
        self.plot_turn.showGrid(x=True, y=True, alpha=0.2)
        self.plot_turn.plot(self.smooth_path[:, 0], self.smooth_path[:, 1], pen=pg.mkPen('r', width=3))

        color_list = [
            (31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40),
            (148, 103, 189), (140, 86, 75), (227, 119, 194), (127, 127, 127)
        ]

        for idx, (i, tp) in enumerate(self.turning_points.items()):
            color = color_list[idx % len(color_list)]
            pen = pg.mkPen(color, width=2)

            start_pt = tp["start"]["point"]
            start_slope = tp["start"]["slope"]
            x0, y0 = start_pt
            self.plot_turn.plot([x0], [y0], pen=None, symbol='o', symbolBrush=color, symbolSize=10)

            if np.isinf(start_slope):
                self.plot_turn.plot([x0, x0], [y0, y0 + SLOPE_LENGTH], pen=pen)
            else:
                dx = SLOPE_LENGTH
                dy = start_slope * dx
                self.plot_turn.plot([x0, x0 + dx], [y0, y0 + dy], pen=pen)

            if "end" in tp:
                end_pt = tp["end"]["point"]
                end_slope = tp["end"]["slope"]
                x1, y1 = end_pt
                self.plot_turn.plot([x1], [y1], pen=None, symbol='o', symbolBrush=color, symbolSize=10)

                if np.isinf(end_slope):
                    self.plot_turn.plot([x1, x1], [y1, y1 - SLOPE_LENGTH], pen=pen)
                else:
                    dx = -SLOPE_LENGTH
                    dy = end_slope * dx
                    self.plot_turn.plot([x1, x1 + dx], [y1, y1 + dy], pen=pen)

                intersection = slope_intersection(start_pt, start_slope, end_pt, end_slope)
                if intersection is not None:
                    ix, iy = intersection
                    self.plot_turn.plot([ix], [iy], pen=None, symbol='o', symbolBrush='y', symbolSize=9)

        # 4) MODIFIED PATH
        self.plot_modified = self.win.addPlot(title="New Path With Turning Point")
        self.plot_modified.setXRange(0, GRID_W)
        self.plot_modified.setYRange(0, GRID_H)
        self.plot_modified.setAspectLocked(True)
        self.plot_modified.showGrid(x=True, y=True, alpha=0.2)
        self.plot_modified.plot(self.modified_path[:, 0], self.modified_path[:, 1], pen=pg.mkPen('r', width=3))

        for tp in self.turning_points.values():
            if "end" not in tp:
                continue

            start_pt = tp["start"]["point"]
            start_slope = tp["start"]["slope"]
            end_pt = tp["end"]["point"]
            end_slope = tp["end"]["slope"]

            intersection = slope_intersection(start_pt, start_slope, end_pt, end_slope)
            if intersection is None:
                continue

            ix, iy = intersection
            self.plot_modified.plot([start_pt[0], ix], [start_pt[1], iy], pen=pg.mkPen('b', width=2))
            self.plot_modified.plot([ix, end_pt[0]], [iy, end_pt[1]], pen=pg.mkPen('r', width=2))
            self.plot_modified.plot([ix], [iy], pen=None, symbol='o', symbolBrush='y', symbolSize=10)

        # 5) MODE DECIDER
        self.win.nextRow()
        self.plot_mode = self.win.addPlot(title="Path Segmentation")
        self.plot_mode.setXRange(0, GRID_W)
        self.plot_mode.setYRange(0, GRID_H)
        self.plot_mode.setAspectLocked(True)
        self.plot_mode.showGrid(x=True, y=True, alpha=0.2)
        self.plot_mode.plot(self.modified_path[:, 0], self.modified_path[:, 1], pen=pg.mkPen('g', width=3))
        self.plot_mode.plot([self.start[0]], [self.start[1]], pen=None, symbol='o', symbolBrush='g', symbolSize=12)
        self.plot_mode.plot([self.goal[0]], [self.goal[1]], pen=None, symbol='o', symbolBrush='r', symbolSize=12)

        last_tip_index = 0
        tip_i = 1

        for inter in self.turn_intersections:
            distances = np.linalg.norm(self.modified_path - inter, axis=1)
            tip_index = int(np.argmin(distances))
            mid_index = (last_tip_index + tip_index) // 2
            p = self.modified_path[mid_index]

            add_text(self.plot_mode, p[0], p[1], "ACK", color='darkgreen')
            add_text(self.plot_mode, inter[0], inter[1] + 2, f"TIP{tip_i}", color=(255, 140, 0))
            self.plot_mode.plot([inter[0]], [inter[1]], pen=None, symbol='o', symbolBrush=(255, 165, 0), symbolSize=11)

            last_tip_index = tip_index
            tip_i += 1

        mid_index = (last_tip_index + len(self.modified_path) - 1) // 2
        p = self.modified_path[mid_index]
        add_text(self.plot_mode, p[0], p[1], "ACK", color='darkgreen')

        # 6) ANIMATION
        self.plot_anim = self.win.addPlot(title="Robot Motion Animation")
        self.plot_anim.setXRange(0, GRID_W)
        self.plot_anim.setYRange(0, GRID_H)
        self.plot_anim.setAspectLocked(True)
        self.plot_anim.showGrid(x=True, y=True, alpha=0.2)
        self.plot_anim.plot(self.modified_path[:, 0], self.modified_path[:, 1], pen=pg.mkPen('g', width=2))

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

    def setup_animation(self):
        if len(self.modified_path) >= 2:
            dx0 = self.modified_path[1][0] - self.modified_path[0][0]
            dy0 = self.modified_path[1][1] - self.modified_path[0][1]
            self.theta = np.arctan2(dy0, dx0)
        else:
            self.theta = 0.0

        self.vx_current = 0.0
        self.vx_target = BOT_SPEED
        self.accel = BOT_ACCEL

        self.tip_set = set(self.tip_indices)
        self.path_index = 0
        self.heading_len = 2.0

        self.mode = "ACK"
        self.turn_target_theta = None

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.animation_step)
        self.timer.start(FRAME_DELAY_FAST_MS)

    def set_timer_delay(self, ms):
        self.timer.start(int(ms))

    def animation_step(self):
        if self.path_index >= len(self.modified_path) - 1:

            self.timer.stop()

            # wait 1 second then restart simulation
            QtCore.QTimer.singleShot(1000, self.restart_simulation)

            return

        p = self.modified_path[self.path_index]

        if self.mode == "ACK":
            p_next = self.modified_path[self.path_index + 1]

            dx = p_next[0] - p[0]
            dy = p_next[1] - p[1]

            desired_theta = np.arctan2(dy, dx)
            omega_cmd = angle_wrap(desired_theta - self.theta)

            self.vx_current += self.accel * (self.vx_target - self.vx_current)
            vx_cmd = self.vx_current

            wheels = mecanum_ack(vx_cmd, omega_cmd)

            dtheta = angle_wrap(desired_theta - self.theta)
            turn_rate = 0.15 if self.path_index == 0 else 0.25
            self.theta += turn_rate * dtheta

            self.update_robot_plot(p, self.theta)

            theta_deg = np.degrees(self.theta) % 360
            self.info_text.setText(
                f"ACK MODE\n"
                f"w1={wheels[0]:.2f}\n"
                f"w2={wheels[1]:.2f}\n"
                f"w3={wheels[2]:.2f}\n"
                f"w4={wheels[3]:.2f}\n"
                f"theta={theta_deg:.1f}°\n"
                f"idx={self.path_index}"
            )
            self.info_text.setPos(p[0] + 3, p[1] + 3)

            next_tips = [t for t in self.tip_indices if t >= self.path_index]
            if next_tips:
                remaining = min(next_tips) - self.path_index
            else:
                remaining = 999999

            if remaining < TIP_STOP_RADIUS:
                self.set_timer_delay(FRAME_DELAY_SLOW_MS)
            elif remaining < TIP_SLOW_RADIUS:
                self.set_timer_delay(FRAME_DELAY_MED_MS)
            else:
                self.set_timer_delay(FRAME_DELAY_FAST_MS)

            if self.path_index in self.tip_set:
                if self.path_index < len(self.modified_path) - 1:
                    dx_tip = self.modified_path[self.path_index + 1][0] - self.modified_path[self.path_index][0]
                    dy_tip = self.modified_path[self.path_index + 1][1] - self.modified_path[self.path_index][1]
                    self.turn_target_theta = np.arctan2(dy_tip, dx_tip)
                else:
                    self.turn_target_theta = self.theta

                self.mode = "TIP"
                self.vx_current = 0.0
                self.set_timer_delay(TIP_DELAY_MS)
                return

            self.path_index += 1

        elif self.mode == "TIP":
            dtheta = angle_wrap(self.turn_target_theta - self.theta)

            if abs(dtheta) < TIP_STOP_TOL:
                self.theta = self.turn_target_theta
                self.update_robot_plot(p, self.theta)
                self.mode = "ACK"
                self.path_index += 1
                self.set_timer_delay(FRAME_DELAY_FAST_MS)
                return

            self.theta += TIP_TURN_RATE * dtheta
            wheels_tip = mecanum_tip(dtheta)

            self.update_robot_plot(p, self.theta)

            theta_deg = np.degrees(self.theta) % 360
            self.info_text.setText(
                f"TIP ROTATION\n"
                f"w1={wheels_tip[0]:.2f}\n"
                f"w2={wheels_tip[1]:.2f}\n"
                f"w3={wheels_tip[2]:.2f}\n"
                f"w4={wheels_tip[3]:.2f}\n"
                f"theta={theta_deg:.1f}°\n"
                f"idx={self.path_index}"
            )
            self.info_text.setPos(p[0] + 3, p[1] + 3)

    def update_robot_plot(self, p, theta):
        self.robot_dot.setData([p[0]], [p[1]])

        hx = p[0] + self.heading_len * np.cos(theta)
        hy = p[1] + self.heading_len * np.sin(theta)
        self.heading_line.setData([p[0], hx], [p[1], hy])

    def run(self):
        self.win.show()
        self.app.exec_()


def main():
    data = None

    for _ in range(20):
        data = build_sim_data()
        if data is not None:
            break

    if data is None:
        print("No path found after several tries")
        return

    print("Analysis points:", len(data["robot_positions"]))
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
    print("ACK")
    for i in range(len(data["turn_intersections"])):
        print(f"TIP{i+1}")
        print(f"ACK{i+1}")

    sim = SimulationWindow(data)
    sim.run()


if __name__ == "__main__":
    main()