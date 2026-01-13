# encoder_local_map.py
import time
import threading
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib

# We only read what encoder_processing already computes (latest_velocity).
# No serial, no re-computing from encoders here.
import encoder_processing as ep  # Ensure encoder_processing is imported

# Global pose state (robot-local map)
robot_position = {
    'x': 0.0,
    'y': 0.0,
    'angle': 0.0,  # radians
}


class EncoderLocalMap:
    """
    Background integrator that:
      - polls ep.latest_velocity (produced by encoder_processing)
      - integrates pose using update_robot_position()
      - exposes getters for pose and velocity
    """

    def __init__(self, poll=0.05, debug=False):
        self.poll = float(poll)
        self.debug = bool(debug)
        self._running = threading.Event()
        self._running.set()

        # Internal timing to compute delta_t between updates
        self._last_t = time.time()
        self._last_seen_t = None
        self._last_warn = 0.0

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        # Hold matplotlib objects if plotting is started
        self._fig = None
        self._ani = None

    # -------- Public API --------
    def get_pose(self):
        return dict(robot_position)

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)

    # Optional: start plot from the instance
    def start_live_plot(self, interval_ms=100, history_len=5000, title="Encoder Local Map", show=True, view_size=20.0, center=(0.0, 0.0)):
        fig, ani = start_live_plot(self, interval_ms=interval_ms, history_len=history_len, title=title, show=show, view_size=view_size, center=center)
        self._fig, self._ani = fig, ani  # keep refs alive
        return fig, ani

    # -------- Worker --------
    def _loop(self):
        if self.debug:
            print("[LocalMap] started (listening to encoder_processing.robot_position)", flush=True)

        while self._running.is_set():
            # Get the robot position from encoder_processing
            pos = getattr(ep, 'robot_position', None)
            if not isinstance(pos, dict):
                self._warn("encoder_processing.robot_position not found – ensure it is defined and updated", every=2.0)
                time.sleep(self.poll)
                continue

            # Extract x, y, and angle from the robot position
            x = float(pos.get('x', 0.0))
            y = float(pos.get('y', 0.0))
            angle = float(pos.get('angle', 0.0))

            # Debug print for position updates
            if self.debug:
                print(f"[LocalMap] x={x:.3f} y={y:.3f} angle={angle:.3f}", flush=True)

            # Update the global robot position for plotting
            robot_position['x'] = x
            robot_position['y'] = y
            robot_position['angle'] = angle

            time.sleep(self.poll)

    def _warn(self, msg, every=2.0):
        """Rate-limited warning prints when debug=True."""
        if not self.debug:
            return
        now = time.time()
        if now - self._last_warn >= float(every):
            print(f"[LocalMap] {msg}", flush=True)
            self._last_warn = now

def start_live_plot(local_map, interval_ms=100, history_len=20, title="Encoder Local Map", show=True, view_size=2, center=(0.0, 0.0)):
    """
    Launch a live, non-blocking plot that tracks the robot pose over time.
    - local_map: an EncoderLocalMap instance
    - interval_ms: animation period in milliseconds
    - history_len: max points to retain in the path
    - title: figure title
    - show: if False, do not enable interactive mode or call plt.show()
    - view_size: half-width/half-height of the square view (meters)
    - center: (x,y) center of the fixed view
    Returns (fig, ani) so the caller can keep references if desired.
    """
    # Only enable interactive mode / show window if requested and backend supports it
    backend = matplotlib.get_backend().lower()
    interactive_ok = bool(show) and backend != "agg"

    if interactive_ok:
        plt.ion()
    else:
        # ensure non-interactive when not showing
        plt.ioff()

    fig, ax = plt.subplots()
    try:
        if interactive_ok:
            fig.canvas.manager.set_window_title(title)
    except Exception:
        # Some backends (or headless) don't support window titles; ignore.
        pass
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)
    ax.set_aspect("equal", adjustable="box")

    # set a large fixed view centered at `center` with half-range `view_size`
    cx, cy = float(center[0]), float(center[1])
    half = float(view_size)
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)

    # path line + current position dot
    path_line, = ax.plot([], [], lw=2)
    pose_dot,  = ax.plot([], [], "o", ms=5)

    xs, ys = [], []

    def init():
        path_line.set_data([], [])
        pose_dot.set_data([], [])
        # keep the fixed limits (already set)
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
        return path_line, pose_dot

    def update(_frame):
        pose = local_map.get_pose()
        x, y = float(pose["x"]), float(pose["y"])

        xs.append(x); ys.append(y)
        if len(xs) > history_len:
            del xs[:len(xs) - history_len]
            del ys[:len(ys) - history_len]

        path_line.set_data(xs, ys)
        pose_dot.set_data([x], [y])

        # fixed view: do not expand limits — keep centered and constant
        return path_line, pose_dot

    ani = animation.FuncAnimation(fig, update, init_func=init, interval=interval_ms, blit=True)
    if interactive_ok:
        plt.show(block=False)   # non-blocking so run_all can keep running

    # store on the instance to prevent GC if caller drops refs
    local_map._fig = fig
    local_map._ani = ani
    return fig, ani

# Helper for run_all.py (backward compatible signature)
def create_and_run(shared_data=None, poll=0.05, debug=False):
    """
    Accepts shared_data for API uniformity with other modules (unused here).
    """
    return EncoderLocalMap(poll=poll, debug=debug)

if __name__ == "__main__":
    print("Run via run_all.py so it starts alongside encoder_processing.")
