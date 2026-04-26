import threading
import time
import numpy as np
from obstacle_grid_processing import fused_pose


class SemanticMapper:

    def __init__(self, vision, lidar_proc, poll=0.2):
        self.vision = vision
        self.shared = vision.shared
        self.lidar_proc = lidar_proc
        self.poll = poll

        if not hasattr(self.shared, "semantic_objects"):
            self.shared.semantic_objects = []

        if not hasattr(self.shared, "semantic_lock"):
            self.shared.semantic_lock = threading.Lock()

        self._running = threading.Event()
        self._running.set()

        print("[Semantic] mapper started", flush=True)

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        while self._running.is_set():
            try:
                with self.shared.vision_lock:
                    tracks = [dict(t) for t in self.shared.vision_tracks]

                for tr in tracks:
                    if tr.get("conf", 0.0) < 0.4:
                        continue

                    if "yaw_ctr" not in tr:
                        continue

                    if "id" not in tr or "label" not in tr:
                        continue

                    angle = np.deg2rad(tr["yaw_ctr"])
                    dist = self.lidar_distance(angle)

                    if dist is None:
                        continue

                    xr = dist * np.cos(angle)
                    yr = dist * np.sin(angle)

                    X, Y = self.robot_to_world(xr, yr)

                    self.store_object(
                        track_id=tr["id"],
                        label=tr["label"],
                        X=X,
                        Y=Y,
                        conf=tr["conf"],
                    )

                self.cleanup_stale_objects(max_age=5.0)

            except Exception as e:
                print("[Semantic ERROR]", e, flush=True)

            time.sleep(self.poll)

    def lidar_distance(self, angle):
        scan = self.lidar_proc.get_latest_scan()

        if scan is None:
            return None

        angles = scan.angles_rad
        ranges = scan.ranges_m

        if angles is None or ranges is None or len(angles) == 0 or len(ranges) == 0:
            return None

        idx = np.argmin(np.abs(angles - angle))

        window = ranges[max(0, idx-2):idx+3]
        window = window[np.isfinite(window)]

        if len(window) == 0:
            return None

        dist = float(np.median(window))

        if np.isnan(dist) or np.isinf(dist) or dist <= 0.0:
            return None

        return dist

    def robot_to_world(self, xr, yr):
        x = fused_pose["x"]
        y = fused_pose["y"]
        theta = fused_pose["theta"]

        X = x + xr * np.cos(theta) - yr * np.sin(theta)
        Y = y + xr * np.sin(theta) + yr * np.cos(theta)

        return X, Y

    def store_object(self, track_id, label, X, Y, conf):
        now = time.time()

        MERGE_DIST = 1.0   # meters
        MAX_JUMP = 2.0     # reject teleport jumps

        with self.shared.semantic_lock:

            # -------------------------------------------------
            # 1️⃣ Spatial merge FIRST (fixes YOLO ID resets)
            # -------------------------------------------------
            for existing in self.shared.semantic_objects:

                if existing["label"] != label:
                    continue

                dx = existing["x"] - X
                dy = existing["y"] - Y
                d = np.hypot(dx, dy)

                if d < MERGE_DIST:

                    # reject impossible jumps
                    if d > MAX_JUMP:
                        return

                    existing["track_id"] = track_id
                    existing["x"] = X
                    existing["y"] = Y
                    existing["conf"] = conf
                    existing["t"] = now

                    print(f"[Semantic MERGE] {label}#{track_id} at ({X:.2f},{Y:.2f}) conf={conf:.2f}", flush=True)
                    return


            # -------------------------------------------------
            # 2️⃣ Track ID match fallback
            # -------------------------------------------------
            for existing in self.shared.semantic_objects:

                if existing["track_id"] == track_id and existing["label"] == label:

                    existing["x"] = X
                    existing["y"] = Y
                    existing["conf"] = conf
                    existing["t"] = now

                    print(f"[Semantic UPDATE] {label}#{track_id} at ({X:.2f},{Y:.2f}) conf={conf:.2f}", flush=True)
                    return


            # -------------------------------------------------
            # 3️⃣ Create new semantic object
            # -------------------------------------------------
            obj = {
                "track_id": track_id,
                "label": label,
                "x": X,
                "y": Y,
                "conf": conf,
                "t": now,
            }

            self.shared.semantic_objects.append(obj)

            print(f"[Semantic NEW] {label}#{track_id} at ({X:.2f},{Y:.2f}) conf={conf:.2f}", flush=True)
            
    def cleanup_stale_objects(self, max_age=5.0):
        now = time.time()

        with self.shared.semantic_lock:
            self.shared.semantic_objects = [
                obj for obj in self.shared.semantic_objects
                if (now - obj["t"]) <= max_age
            ]

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self.thread.join(timeout=join_timeout)


def create_and_run(vision, lidar_proc, poll=0.2):
    return SemanticMapper(vision, lidar_proc, poll=poll)