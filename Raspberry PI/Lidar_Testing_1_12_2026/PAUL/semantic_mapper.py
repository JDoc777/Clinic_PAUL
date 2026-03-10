import threading
import time
import numpy as np
from obstacle_grid_processing import fused_pose


class SemanticMapper:

    def __init__(self, shared, lidar_proc, poll=0.2):
        self.shared = shared
        self.lidar_proc = lidar_proc
        self.poll = poll

        self._running = threading.Event()
        self._running.set()

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):

        while self._running.is_set():

            try:

                if not hasattr(self.shared, "vision_tracks"):
                    time.sleep(self.poll)
                    continue

                with self.shared.vision_lock:
                    tracks = list(self.shared.vision_tracks)

                for tr in tracks:

                    if tr["conf"] < 0.5:
                        continue

                    angle = np.deg2rad(tr["yaw_ctr"])

                    dist = self.lidar_distance(angle)

                    if dist is None:
                        continue

                    xr = dist * np.cos(angle)
                    yr = dist * np.sin(angle)
                    print(f"Robot frame: xr={xr:.2f}  yr={yr:.2f}")

                    X, Y = self.robot_to_world(xr, yr)

                    self.store_object(tr["id"], tr["label"], X, Y, tr["conf"])
            except Exception:
                pass

            time.sleep(self.poll)

    def lidar_distance(self, angle):

        scan = self.lidar_proc.get_latest_scan()

        if scan is None:
            return None

        angles = scan.angles_rad
        ranges = scan.ranges_m

        idx = np.argmin(np.abs(angles - angle))

        return ranges[idx]

    def robot_to_world(self, xr, yr):

        x = fused_pose["x"]
        y = fused_pose["y"]
        theta = fused_pose["theta"]

        X = x + xr*np.cos(theta) - yr*np.sin(theta)
        Y = y + xr*np.sin(theta) + yr*np.cos(theta)
        print(f"World: X={X:.2f} Y={Y:.2f}")

        return X, Y

    def store_object(self, track_id, label, X, Y, conf):

        now = time.time()

        with self.shared.semantic_lock:

            for existing in self.shared.semantic_objects:
                if existing["track_id"] == track_id and existing["label"] == label:
                    existing["x"] = X
                    existing["y"] = Y
                    existing["conf"] = conf
                    existing["t"] = now
                    print(f"[Semantic UPDATE] {label}#{track_id} at ({X:.2f},{Y:.2f}) conf={conf:.2f}")
                    return

            obj = {
                "track_id": track_id,
                "label": label,
                "x": X,
                "y": Y,
                "conf": conf,
                "t": now
            }

            self.shared.semantic_objects.append(obj)
            print(f"[Semantic NEW] {label}#{track_id} at ({X:.2f},{Y:.2f}) conf={conf:.2f}")
    
def create_and_run(shared_data, lidar_proc, poll=0.2):

    if not hasattr(shared_data, "semantic_objects"):
        shared_data.semantic_objects = []

    if not hasattr(shared_data, "semantic_lock"):
        shared_data.semantic_lock = threading.Lock()

    mapper = SemanticMapper(shared_data, lidar_proc, poll=poll)

    return mapper