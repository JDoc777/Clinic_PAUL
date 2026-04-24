# sec2_plot_real.py

import time
import threading
import numpy as np
from LocomotionSIM2 import build_section2_segmented_path_from_smooth_path


class RealSEC2:

    def __init__(self, grid, poll=0.1):
        self.grid = grid
        self.poll = poll
        self.running = True

        self.latest_data = None


    def compute(self):

        smooth_path = getattr(self.grid, "current_path_smooth", None)

        if smooth_path is None or len(smooth_path) < 3:
            return None

        smooth_path = np.asarray(smooth_path)

        raw_path = getattr(self.grid, "current_path_world", None)
        if raw_path is None:
            raw_path = smooth_path

        data = build_section2_segmented_path_from_smooth_path(
            smooth_path_m=smooth_path,
            raw_path_m=np.asarray(raw_path),
            grid=self.grid.grid.copy(),
            start_m=smooth_path[0],
            goal_m=smooth_path[-1]
        )

        return data


    def run(self):

        while self.running:

            data = self.compute()

            if data is not None:
                self.latest_data = data

            time.sleep(self.poll)


def create_and_run(grid, poll=0.1):

    sec2 = RealSEC2(grid, poll)

    thread = threading.Thread(target=sec2.run, daemon=True)
    thread.start()

    return sec2