import math
import time
import threading

from pathfinding import add_headings

class LocalMotionModeProcessor:

    def __init__(self, grid, poll=0.05):

        self.grid = grid
        self.poll = poll

        self._running = threading.Event()
        self._running.set()

        self.last_path_signature = None

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
    
    def loop(self):
        while self._running.is_set():
            try:
                self.process()
            except Exception as e:
                print(f"[LocalMotionModeProcessor] error: {e}")

            time.sleep(self.poll)

    def process(self):

        path = self.grid.current_path_smooth

        # no path yet
        if path is None or len(path) < 3:
            return

        signature = (
            len(path),
            round(path[0][0],2),
            round(path[0][1],2),
            round(path[-1][0],2),
            round(path[-1][1],2)
        )

        if signature == self.last_path_signature:
            return

        self.last_path_signature = signature
        # run annotation
        drive_modes, turn_points = annotate_modes(path)

        # store results back in grid
        self.grid.current_path_modes = drive_modes
        self.grid.current_turn_points = turn_points

        print(f"[MotionModes] Annotated path with {len(path)} points")


# =========================
# CONFIG
# =========================
R_MIN_CELLS = 6.0
TURN_HEADING_DEG = 50.0
CRAB_ANGLE_DEG = 15.0


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
    - CRAB if local motion is sideways relative to heading
    """
    if i <= 0 or i >= len(path_xy) - 1:
        return "ACKERMANN"

    x0, y0 = path_xy[i - 1]
    x1, y1 = path_xy[i]
    x2, y2 = path_xy[i + 1]

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
    Input:
        path_xy = [(x, y), ...]

    Returns:
        drive_modes : list[str]
        turn_points : list[int]
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

        # pivot-turn event
        if R < R_MIN_CELLS or abs(dtheta) > turn_heading:
            turn_candidates.append(i)

        # travel mode for non-pivot motion
        drive_modes[i] = classify_drive_mode(path_xy, headings, i)

    turn_points = merge_nearby_indices(turn_candidates)

    if n >= 2:
        drive_modes[0] = drive_modes[1]
        drive_modes[-1] = drive_modes[-2]

    return drive_modes, turn_points

def create_and_run(grid, poll=0.05):
    return LocalMotionModeProcessor(grid, poll)