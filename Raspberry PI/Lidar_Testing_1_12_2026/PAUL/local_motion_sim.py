import random
import numpy as np
import matplotlib.pyplot as plt

from pathfinding import astar, inflate_obstacles, bezier


# =========================
# SIM CONFIG
# =========================
GRID_W = 140
GRID_H = 100
CELL_SIZE_M = 0.10
ROBOT_RADIUS_M = 0.25
SLOPE_LENGTH = 20

L = 12.0        # lookahead distance along path
R_min = 30      # threshold for detecting turning


def slope_intersection(p1, m1, p2, m2):
    x1, y1 = p1
    x2, y2 = p2

    # vertical line cases
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

    # parallel lines
    if abs(m1 - m2) < 1e-6:
        return None

    # solve intersection
    x = (m1 * x1 - m2 * x2 + y2 - y1) / (m1 - m2)
    y = y1 + m1 * (x - x1)

    return x, y


# =========================
# RANDOM OBSTACLE GRID
# =========================
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


# =========================
# RANDOM START / GOAL
# =========================
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
# ARC LENGTH
# =========================
def compute_arc_lengths(path):
    s = [0.0]

    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        s.append(s[-1] + np.sqrt(dx * dx + dy * dy))

    return np.array(s)


# =========================
# LOOKAHEAD POINT
# =========================
def find_lookahead_point(path, arc_len, robot_index, L):
    target_s = arc_len[robot_index] + L

    for i in range(robot_index, len(path)):
        if arc_len[i] >= target_s:
            return path[i], i

    return None, None


# =========================
# MAIN
# =========================
def main():
    grid = make_big_obstacle_grid()
    start, goal = pick_start_goal(grid)

    inflated = inflate_obstacles(grid, ROBOT_RADIUS_M, CELL_SIZE_M)
    path_grid = astar(inflated, start, goal)

    if path_grid is None or len(path_grid) < 2:
        print("No path found")
        return

    path = [(float(x), float(y)) for (x, y) in path_grid]

    # =========================
    # BEZIER SMOOTHING
    # =========================
    if len(path) >= 3:
        smooth_path = np.asarray(bezier(path, n=max(80, len(path) * 4)))
    else:
        smooth_path = np.asarray(path)

    # =========================
    # ARC LENGTH
    # =========================
    arc_len = compute_arc_lengths(smooth_path)

    # =========================
    # COMPUTE R ALONG PATH
    # =========================
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

    print("Analysis points:", len(robot_positions))

    # =========================
    # DETECT TURNING SEGMENTS
    # =========================
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

        # START TURN
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

        # END TURN
        elif not tight_turn and in_turn:
            turning_points[turn_index]["end"] = {
                "point": robot_pos,
                "slope": slope,
                "intersection": lookahead
            }
            turn_index += 1
            in_turn = False

    # handle case where turn continues to end of path
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

    # =========================
    # PRINT DICTIONARY
    # =========================
    print("\nTurning Points:\n")

    for i, tp in turning_points.items():
        print(f"TurningPoint[{i}]")
        print("  start:")
        print(f"    point = {tp['start']['point']}")
        print(f"    slope = {tp['start']['slope']}")
        print(f"    R = {tp['start']['R']}")

        if "end" in tp:
            print("  end:")
            print(f"    point = {tp['end']['point']}")
            print(f"    slope = {tp['end']['slope']}")

    # =========================
    # STORE INTERSECTION POINTS
    # =========================
    turn_intersections = []

    for tp in turning_points.values():

        if "end" not in tp:
            continue

        start_pt = tp["start"]["point"]
        start_slope = tp["start"]["slope"]

        end_pt = tp["end"]["point"]
        end_slope = tp["end"]["slope"]

        intersection = slope_intersection(
            start_pt,
            start_slope,
            end_pt,
            end_slope
        )

        if intersection is not None:
            turn_intersections.append(intersection)
    
    
    # =========================
    # BUILD MODIFIED PATH (SAFE)
    # =========================
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
            end_pt   = tuple(tp["end"]["point"])

            start_slope = tp["start"]["slope"]
            end_slope   = tp["end"]["slope"]

            # detect start of turn
            if np.linalg.norm(np.array(p) - np.array(start_pt)) < 0.5:

                intersection = slope_intersection(
                    start_pt,
                    start_slope,
                    end_pt,
                    end_slope
                )

                modified_path.append(start_pt)

                if intersection is not None:
                    modified_path.append(intersection)

                modified_path.append(end_pt)

                # skip ahead until end_pt in original path
                while i < len(smooth_path):

                    if np.linalg.norm(
                        smooth_path[i] - np.array(end_pt)
                    ) < 0.5:
                        break

                    i += 1

                replaced = True
                break

        if not replaced:
            modified_path.append(p)

        i += 1

    modified_path = np.array(modified_path)

    # =========================
    # PLOT
    # =========================
    fig, axs = plt.subplots(2, 2, figsize=(18, 10))

    ax_map = axs[0, 0]
    ax_path = axs[0, 1]
    ax_turn = axs[1, 0]



    sx = smooth_path[:, 0]
    sy = smooth_path[:, 1]

    # =========================
    # MAP
    # =========================
    ax_map.imshow(grid, cmap="gray_r", origin="lower")
    ax_map.set_title("Random Obstacles + A* + Bezier")
    ax_map.set_xlim(0, GRID_W)
    ax_map.set_ylim(0, GRID_H)
    ax_map.set_aspect("equal")

    px = [p[0] for p in path]
    py = [p[1] for p in path]

    ax_map.plot(px, py, "y--", linewidth=1.5)
    ax_map.plot(sx, sy, "r-", linewidth=2.5)

    ax_map.scatter(start[0], start[1], s=120, color="lime")
    ax_map.scatter(goal[0], goal[1], s=120, color="red")

    # =========================
    # PATH ANALYSIS
    # =========================
    ax_path.set_title("Path Analysis")
    ax_path.set_xlim(0, GRID_W)
    ax_path.set_ylim(0, GRID_H)
    ax_path.set_aspect("equal")
    ax_path.grid(True, alpha=0.3)

    ax_path.plot(sx, sy, "r-", linewidth=3)

    for i, tp in turning_points.items():
        start_point = tp["start"]["point"]
        start_R = tp["start"]["R"]
        intersection = tp["start"]["intersection"]

        ax_path.scatter(start_point[0], start_point[1], color="blue", s=120)
        ax_path.scatter(intersection[0], intersection[1], color="orange", s=140)

        circle = plt.Circle(
            start_point,
            L,
            fill=False,
            linestyle="--",
            linewidth=2,
            color="green"
        )
        ax_path.add_patch(circle)

        ax_path.text(
            start_point[0] + 3,
            start_point[1] + 3,
            f"R={start_R:.1f}",
            fontsize=10,
            bbox=dict(facecolor="white", alpha=0.8)
        )

        if "end" in tp:
            end_point = tp["end"]["point"]
            ax_path.scatter(end_point[0], end_point[1], color="purple", s=120)

    # =========================
    # TURNING POINT CALCULATION
    # =========================
    ax_turn.set_title("Turning Point Calculation")
    ax_turn.set_xlim(0, GRID_W)
    ax_turn.set_ylim(0, GRID_H)
    ax_turn.set_aspect("equal")
    ax_turn.grid(True, alpha=0.3)

    ax_turn.plot(sx, sy, "r-", linewidth=3)

    colors = plt.cm.tab10(np.linspace(0, 1, max(1, len(turning_points))))

    for (i, tp), color in zip(turning_points.items(), colors):
        start_pt = tp["start"]["point"]
        start_slope = tp["start"]["slope"]

        x0, y0 = start_pt
        ax_turn.scatter(x0, y0, color=color, s=120)

        dx = SLOPE_LENGTH
        dy = start_slope * dx if not np.isinf(start_slope) else 0

        if np.isinf(start_slope):
            ax_turn.plot(
                [x0, x0],
                [y0, y0 + SLOPE_LENGTH],
                color=color,
                linestyle="--",
                linewidth=2
            )
        else:
            ax_turn.plot(
                [x0, x0 + dx],
                [y0, y0 + dy],
                color=color,
                linestyle="--",
                linewidth=2
            )

        if "end" in tp:
            end_pt = tp["end"]["point"]
            end_slope = tp["end"]["slope"]

            x1, y1 = end_pt
            ax_turn.scatter(x1, y1, color=color, s=120)

            dx = -SLOPE_LENGTH
            dy = end_slope * dx if not np.isinf(end_slope) else 0

            if np.isinf(end_slope):
                ax_turn.plot(
                    [x1, x1],
                    [y1, y1 - SLOPE_LENGTH],
                    color=color,
                    linestyle="--",
                    linewidth=2
                )
            else:
                ax_turn.plot(
                    [x1, x1 + dx],
                    [y1, y1 + dy],
                    color=color,
                    linestyle="--",
                    linewidth=2
                )

            intersection = slope_intersection(start_pt, start_slope, end_pt, end_slope)

            if intersection is not None:
                ix, iy = intersection
                ax_turn.scatter(ix, iy, color="gold", s=40, zorder=10)

    # =========================
    # INTERSECTION SUMMARY
    # =========================
    ax_intersections = axs[1,1]

    ax_intersections.set_title("Path + Intersection Points")
    ax_intersections.set_xlim(0, GRID_W)
    ax_intersections.set_ylim(0, GRID_H)
    ax_intersections.set_aspect("equal")
    ax_intersections.grid(True, alpha=0.3)

    # plot path
    mx = modified_path[:,0]
    my = modified_path[:,1]

    ax_intersections.plot(mx, my, "r-", linewidth=3)

    for tp in turning_points.values():

        if "end" not in tp:
            continue

        start_pt = tp["start"]["point"]
        start_slope = tp["start"]["slope"]

        end_pt = tp["end"]["point"]
        end_slope = tp["end"]["slope"]

        intersection = slope_intersection(
            start_pt,
            start_slope,
            end_pt,
            end_slope
        )

        if intersection is None:
            continue

        ix, iy = intersection

        # draw start -> intersection (BLUE)
        if np.isinf(start_slope):

            ax_intersections.plot(
                [start_pt[0], ix],
                [start_pt[1], iy],
                linestyle="--",
                color="blue",
                linewidth=2
            )

        else:

            ax_intersections.plot(
                [start_pt[0], ix],
                [start_pt[1], iy],
                linestyle="--",
                color="red",
                linewidth=2
            )

        # draw intersection -> end (ORANGE)
        ax_intersections.plot(
            [ix, end_pt[0]],
            [iy, end_pt[1]],
            linestyle="--",
            color="red",
            linewidth=2
        )

        # intersection point
        ax_intersections.scatter(
            ix,
            iy,
            color="gold",
            s=60,
            zorder=10
        )

    segments = []

    for i in range(len(modified_path)-1):

        p1 = modified_path[i]
        p2 = modified_path[i+1]

        mode = "ACKERMANN"

        for inter in turn_intersections:

            if (
                np.linalg.norm(p1 - inter) < 0.5 or
                np.linalg.norm(p2 - inter) < 0.5
            ):
                mode = "TURNING_POINT"
                break

        segments.append((p1, p2, mode))

    # =========================
    # BUILD MODE SEQUENCE
    # =========================
    mode_sequence = []

    ack_count = 1
    tip_count = 1

    for i in range(len(modified_path)-1):

        p1 = modified_path[i]
        p2 = modified_path[i+1]

        mode_sequence.append(("ACK", ack_count, p1, p2))
        ack_count += 1

        for inter in turn_intersections:

            if np.linalg.norm(p2 - inter) < 0.5:

                mode_sequence.append(("TIP", tip_count, inter))
                tip_count += 1
                break
            
    print("\nMode Sequence:\n")

    tip_i = 1
    ACK_i = 1

    # first state is always ACK
    print("ACK")

    for _ in turn_intersections:
        print(f"TIP{tip_i}")
        print(f"ACK{ACK_i}")
        tip_i += 1
        ACK_i += 1

    # =========================
    # MODE DECIDER
    # =========================
    fig2, ax_mode = plt.subplots(figsize=(8,6))

    ax_mode.set_title("Mode Decider")

    ax_mode.set_xlim(0, GRID_W)
    ax_mode.set_ylim(0, GRID_H)
    ax_mode.set_aspect("equal")

    ax_mode.grid(True, alpha=0.3)

    mx = modified_path[:,0]
    my = modified_path[:,1]

    # ALL segments = ACKERMANN
    ax_mode.plot(mx, my, color="green", linewidth=3)

    # =========================
    # STATE LABELING
    # =========================

    last_tip_index = 0
    tip_i = 1
    

    for inter in turn_intersections:

        # find closest path index to the TIP
        distances = np.linalg.norm(modified_path - inter, axis=1)
        tip_index = np.argmin(distances)

        # midpoint of ACK region before the TIP
        mid_index = (last_tip_index + tip_index) // 2
        p = modified_path[mid_index]

        ax_mode.text(
            p[0],
            p[1],
            "ACK",
            fontsize=11,
            color="darkgreen",
            ha="center",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="green")
        )

        # label the TIP itself
        ax_mode.text(
            inter[0],
            inter[1] + 2,
            f"TIP{tip_i}",
            fontsize=11,
            color="darkorange",
            ha="center",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="orange")
        )

        last_tip_index = tip_index
        tip_i += 1


    # final ACK after last TIP
    mid_index = (last_tip_index + len(modified_path) - 1) // 2
    p = modified_path[mid_index]

    ax_mode.text(
        p[0],
        p[1],
        "ACK",
        fontsize=11,
        color="darkgreen",
        ha="center",
        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="green")
    )
    # plot TIP points
    for inter in turn_intersections:

        ax_mode.scatter(
            inter[0],
            inter[1],
            color="orange",
            s=120,
            zorder=10
        )

    # start / goal
    ax_mode.scatter(start[0], start[1], color="lime", s=120)
    ax_mode.scatter(goal[0], goal[1], color="red", s=120)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()