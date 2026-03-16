import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication


MAP_W_M = 5.0
MAP_H_M = 5.0


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


def add_text(plot, x, y, text, color='k'):
    t = pg.TextItem(text=text, color=color, anchor=(0.5, 0.5))
    t.setPos(float(x), float(y))
    plot.addItem(t)
    return t


class LocomotionSEC2Plot:
    def __init__(self, data):
        self.data = data

        self.smooth_path = np.asarray(self.data["smooth_path"], dtype=float)
        self.turning_points = self.data["turning_points"]
        self.turn_intersections = self.data["turn_intersections"]
        self.modified_path = np.asarray(self.data["modified_path"], dtype=float)
        self.tip_indices = self.data["tip_indices"]
        self.lookahead_points = self.data["lookahead_points"]
        self.robot_positions = self.data["robot_positions"]
        self.R_values = self.data["R_values"]

        self.app = QApplication.instance()
        if self.app is None:
            self.app = QApplication([])

        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")

        self.win = pg.GraphicsLayoutWidget(title="Locomotion Section 2 Plot")
        self.win.resize(1400, 900)

        self.setup_plots()

    def update(self, data):

        self.data = data

        self.smooth_path = data["smooth_path"]
        self.turning_points = data["turning_points"]
        self.turn_intersections = data["turn_intersections"]
        self.modified_path = data["modified_path"]
        self.tip_indices = data["tip_indices"]
        self.lookahead_points = data["lookahead_points"]
        self.robot_positions = data["robot_positions"]
        self.R_values = data["R_values"]

        self.win.clear()
        self.setup_plots()

    def setup_plots(self):
        # ---------------------------------
        # 1) PATH ANALYSIS
        # ---------------------------------
        self.plot_path = self.win.addPlot(title="Section 2: Path Analysis")
        self.plot_path.setXRange(0, MAP_W_M)
        self.plot_path.setYRange(0, MAP_H_M)
        self.plot_path.setAspectLocked(True)
        self.plot_path.showGrid(x=True, y=True, alpha=0.2)

        if len(self.smooth_path) > 0:
            self.plot_path.plot(
                self.smooth_path[:, 0],
                self.smooth_path[:, 1],
                pen=pg.mkPen('r', width=3)
            )

        for tp in self.turning_points.values():
            start_point = tp["start"]["point"]
            start_R = tp["start"]["R"]
            intersection = tp["start"]["intersection"]

            self.plot_path.plot(
                [start_point[0]], [start_point[1]],
                pen=None, symbol='o', symbolBrush='b', symbolSize=10
            )
            self.plot_path.plot(
                [intersection[0]], [intersection[1]],
                pen=None, symbol='o', symbolBrush=(255, 165, 0), symbolSize=10
            )

            add_text(
                self.plot_path,
                start_point[0] + 0.08,
                start_point[1] + 0.08,
                f"R={start_R:.2f}"
            )

            if "end" in tp:
                end_point = tp["end"]["point"]
                self.plot_path.plot(
                    [end_point[0]], [end_point[1]],
                    pen=None, symbol='o', symbolBrush=(128, 0, 128), symbolSize=10
                )

        # ---------------------------------
        # 2) TURN CALCULATION
        # ---------------------------------
        self.plot_turn = self.win.addPlot(title="Section 2: Turning Point Calculation")
        self.plot_turn.setXRange(0, MAP_W_M)
        self.plot_turn.setYRange(0, MAP_H_M)
        self.plot_turn.setAspectLocked(True)
        self.plot_turn.showGrid(x=True, y=True, alpha=0.2)

        if len(self.smooth_path) > 0:
            self.plot_turn.plot(
                self.smooth_path[:, 0],
                self.smooth_path[:, 1],
                pen=pg.mkPen('r', width=3)
            )

        color_list = [
            (31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40),
            (148, 103, 189), (140, 86, 75), (227, 119, 194), (127, 127, 127)
        ]

        slope_len_m = 0.5

        for idx, (_, tp) in enumerate(self.turning_points.items()):
            color = color_list[idx % len(color_list)]
            pen = pg.mkPen(color, width=2)

            start_pt = tp["start"]["point"]
            start_slope = tp["start"]["slope"]
            x0, y0 = start_pt

            self.plot_turn.plot(
                [x0], [y0],
                pen=None, symbol='o', symbolBrush=color, symbolSize=10
            )

            if np.isinf(start_slope):
                self.plot_turn.plot([x0, x0], [y0, y0 + slope_len_m], pen=pen)
            else:
                dx = slope_len_m
                dy = start_slope * dx
                self.plot_turn.plot([x0, x0 + dx], [y0, y0 + dy], pen=pen)

            if "end" in tp:
                end_pt = tp["end"]["point"]
                end_slope = tp["end"]["slope"]
                x1, y1 = end_pt

                self.plot_turn.plot(
                    [x1], [y1],
                    pen=None, symbol='o', symbolBrush=color, symbolSize=10
                )

                if np.isinf(end_slope):
                    self.plot_turn.plot([x1, x1], [y1, y1 - slope_len_m], pen=pen)
                else:
                    dx = -slope_len_m
                    dy = end_slope * dx
                    self.plot_turn.plot([x1, x1 + dx], [y1, y1 + dy], pen=pen)

                intersection = slope_intersection(start_pt, start_slope, end_pt, end_slope)
                if intersection is not None:
                    ix, iy = intersection
                    self.plot_turn.plot(
                        [ix], [iy],
                        pen=None, symbol='o', symbolBrush='y', symbolSize=9
                    )

        self.win.nextRow()

        # ---------------------------------
        # 3) MODIFIED PATH
        # ---------------------------------
        self.plot_modified = self.win.addPlot(title="Section 2: Modified Path")
        self.plot_modified.setXRange(0, MAP_W_M)
        self.plot_modified.setYRange(0, MAP_H_M)
        self.plot_modified.setAspectLocked(True)
        self.plot_modified.showGrid(x=True, y=True, alpha=0.2)

        if len(self.modified_path) > 0:
            self.plot_modified.plot(
                self.modified_path[:, 0],
                self.modified_path[:, 1],
                pen=pg.mkPen('g', width=3)
            )

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
            self.plot_modified.plot(
                [start_pt[0], ix], [start_pt[1], iy],
                pen=pg.mkPen('b', width=2)
            )
            self.plot_modified.plot(
                [ix, end_pt[0]], [iy, end_pt[1]],
                pen=pg.mkPen('r', width=2)
            )
            self.plot_modified.plot(
                [ix], [iy],
                pen=None, symbol='o', symbolBrush='y', symbolSize=10
            )

        # ---------------------------------
        # 4) PATH SEGMENT LABELS
        # ---------------------------------
        self.plot_mode = self.win.addPlot(title="Section 2: Path Segmentation")
        self.plot_mode.setXRange(0, MAP_W_M)
        self.plot_mode.setYRange(0, MAP_H_M)
        self.plot_mode.setAspectLocked(True)
        self.plot_mode.showGrid(x=True, y=True, alpha=0.2)

        if len(self.modified_path) > 0:
            self.plot_mode.plot(
                self.modified_path[:, 0],
                self.modified_path[:, 1],
                pen=pg.mkPen('m', width=3)
            )

        last_tip_index = 0
        tip_i = 1

        for inter in self.turn_intersections:
            if len(self.modified_path) == 0:
                break

            distances = np.linalg.norm(self.modified_path - inter, axis=1)
            tip_index = int(np.argmin(distances))
            mid_index = (last_tip_index + tip_index) // 2
            p_mid = self.modified_path[mid_index]

            add_text(self.plot_mode, p_mid[0], p_mid[1], "ACK", color='darkgreen')
            add_text(self.plot_mode, inter[0], inter[1] + 0.08, f"TIP{tip_i}", color=(255, 140, 0))

            self.plot_mode.plot(
                [inter[0]], [inter[1]],
                pen=None, symbol='o', symbolBrush=(255, 165, 0), symbolSize=11
            )

            last_tip_index = tip_index
            tip_i += 1

        if len(self.modified_path) > 0:
            mid_index = (last_tip_index + len(self.modified_path) - 1) // 2
            p_mid = self.modified_path[mid_index]
            add_text(self.plot_mode, p_mid[0], p_mid[1], "ACK", color='darkgreen')

    def run(self):
        self.win.show()
        self.app.exec_()