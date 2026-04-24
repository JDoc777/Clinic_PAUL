import json
import time
import threading

from obstacle_grid_processing import fused_pose
import lidar_processing


class RobotLogger:

    def __init__(self, grid, filename="robot_log.jsonl", poll=0.1):
        
        self.grid = grid
        self.file = open(filename, "w")
        self.lock = threading.Lock()
        self.start_time = time.time()

        self.poll = poll
        self._running = threading.Event()
        self._running.set()

        print(f"[LOGGER] Logging to {filename}")

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()


    def loop(self):

        while self._running.is_set():

            data = {}

            # ----------------
            # ROBOT POSE
            # ----------------
            data["pose"] = {
                "x": fused_pose["x"],
                "y": fused_pose["y"],
                "theta": fused_pose["theta"]
            }

            # ----------------
            # NAVIGATION DATA
            # ----------------
            if self.grid is not None:

                data["goal"] = {
                    "x": self.grid.goal_x,
                    "y": self.grid.goal_y
                }

                data["wp_index"] = self.grid.current_wp_index

                data["waypoint"] = {
                    "x": self.grid.wp_x,
                    "y": self.grid.wp_y,
                    "theta": self.grid.wp_theta
                }

                data["path_len"] = (
                    len(self.grid.current_waypoints)
                    if self.grid.current_waypoints else 0
                )

                # ----------------
                # A* PATH
                # ----------------
                if getattr(self.grid, "current_path_grid", None) is not None:
                    data["astar_path"] = self.grid.current_path_grid

                # ----------------
                # SMOOTHED PATH
                # ----------------
                if getattr(self.grid, "current_path_smooth", None):
                    data["smooth_path"] = self.grid.current_path_smooth

                # ----------------
                # GRID SNAPSHOT (ONLY WHEN REQUESTED)
                # ----------------
                if getattr(self.grid, "snapshot_requested", False):

                    data["grid_snapshot"] = self.grid.grid.tolist()

                    # reset request flag
                    self.grid.snapshot_requested = False

                # ----------------
                # INFLATED OBSTACLE GRID
                # ----------------
                if getattr(self.grid, "inflated_grid", None):
                    data["inflated_grid"] = self.grid.inflated_grid.tolist()

            # ----------------
            # LIDAR DATA
            # ----------------
            scan = getattr(lidar_processing, "latest_scan", None)

            if scan is not None:

                data["ranges"] = scan["ranges"]
                data["angles"] = scan["angles"]

            # ----------------
            # WRITE ENTRY
            # ----------------
            entry = {
                "t": time.time() - self.start_time,
                **data
            }

            with self.lock:
                self.file.write(json.dumps(entry) + "\n")
                self.file.flush()

            time.sleep(self.poll)


    def close(self):
        self._running.clear()
        self.thread.join()

        with self.lock:
            self.file.close()


def create_and_run(grid, filename="robot_log.jsonl"):

    logger = RobotLogger(grid, filename)
    return logger