import heapq
import numpy as np
import math
from scipy.interpolate import splprep, splev


# ---------------------------------------------------------
# A* Pathfinding (grid-based)
# ---------------------------------------------------------
def astar(grid, start, goal):
    h, w = grid.shape
    sx, sy = start
    gx, gy = goal

    if not (0 <= sx < w and 0 <= sy < h): return None
    if not (0 <= gx < w and 0 <= gy < h): return None
    if grid[sy, sx] == 1: return None
    if grid[gy, gx] == 1: return None

    open_set = []
    heapq.heappush(open_set, (0, 0.0, (sx, sy)))
    
    came_from = {}
    g_score = {(sx, sy): 0.0}

    neighbors = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    closed_set = set()

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        cx, cy = current

        if current == (gx, gy):
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        if current in closed_set:
            continue
        closed_set.add(current)

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            if grid[ny, nx] == 1:
                continue

            step_cost = math.hypot(dx, dy)
            new_cost = cost + step_cost

            if (nx, ny) not in g_score or new_cost < g_score[(nx, ny)]:
                g_score[(nx, ny)] = new_cost
                priority = new_cost + math.hypot(gx - nx, gy - ny)
                came_from[(nx, ny)] = (cx, cy)
                heapq.heappush(open_set, (priority, new_cost, (nx, ny)))

    return None

def inflate_obstacles(grid, robot_radius_m, cell_size):
    """
    Expand all obstacles outward by robot radius (in meters).
    This marks nearby free cells as 'occupied' so A* avoids them.
    """

    radius_cells = int(robot_radius_m / cell_size)
    h, w = grid.shape

    inflated = grid.copy()

    # loop through each obstacle cell
    for iy in range(h):
        for ix in range(w):
            if grid[iy, ix] == 1:
                # mark neighborhood around it
                for dy in range(-radius_cells, radius_cells + 1):
                    for dx in range(-radius_cells, radius_cells + 1):
                        ny = iy + dy
                        nx = ix + dx
                        if 0 <= ny < h and 0 <= nx < w:
                            inflated[ny, nx] = 1

    return inflated

# ---------------------------------------------------------
# Convert grid coordinates to world coordinates
# ---------------------------------------------------------
def grid_to_world(path_grid, grid_origin, cell_size):
    """
    path_grid   : [(gx, gy), ...]  (gx = x/column, gy = y/row)
    grid_origin : (x0, y0) world coordinates of cell (0,0)
    cell_size   : meters per cell
    """
    x0, y0 = grid_origin
    return [(x0 + gx * cell_size,
             y0 + gy * cell_size) for gx, gy in path_grid]

# ---------------------------------------------------------
# Spline smoothing for nice trajectories
# ---------------------------------------------------------
def smooth_path_spline(path_world, smoothing=0.5, num_points=100):
    if len(path_world) < 3:
        # nothing to really smooth
        return path_world

    x, y = zip(*path_world)
    # ensure unique points for splines
    pts = list(dict.fromkeys(zip(x, y)))
    if len(pts) < 3:
        return pts

    x, y = zip(*pts)
    tck, _ = splprep([x, y], s=smoothing)
    u_fine = np.linspace(0, 1, num_points)
    x_smooth, y_smooth = splev(u_fine, tck)
    return list(zip(x_smooth, y_smooth))

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



# ---------------------------------------------------------
# Heading assignment for robot navigation
# ---------------------------------------------------------
def add_headings(path):
    """
    path: [(x, y), ...]
    returns: [(x, y, theta), ...]
    theta in radians
    """
    n = len(path)
    if n == 0:
        return []
    if n == 1:
        x, y = path[0]
        return [(x, y, 0.0)]

    waypoints = []
    for i in range(n - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        theta = math.atan2(y2 - y1, x2 - x1)
        waypoints.append((x1, y1, theta))

    # last point uses last segment’s heading
    x_last, y_last = path[-1]
    waypoints.append((x_last, y_last, theta))
    return waypoints


# OPTIONAL: keep your navigation_loop (if you use it),
# but it should call astar(), grid_to_world(), smooth_path_spline(), add_headings()
# with the same assumptions as above.
