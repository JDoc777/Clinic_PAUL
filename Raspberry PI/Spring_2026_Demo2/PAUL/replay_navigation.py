import json
import math
import matplotlib.pyplot as plt


LOG_FILE = "robot_run.jsonl"


poses_x = []
poses_y = []

goal_x = None
goal_y = None


plt.ion()
fig, ax = plt.subplots()


with open(LOG_FILE) as f:

    for line in f:

        entry = json.loads(line)

        pose = entry["pose"]

        x = pose["x"]
        y = pose["y"]

        poses_x.append(x)
        poses_y.append(y)

        ax.clear()

        # -------------------
        # ROBOT PATH
        # -------------------
        ax.plot(poses_x, poses_y, "b-", label="robot path")

        # -------------------
        # ROBOT POSITION
        # -------------------
        ax.plot(x, y, "bo")

        # -------------------
        # WAYPOINT
        # -------------------
        if "waypoint" in entry:

            wp = entry["waypoint"]

            ax.plot(
                wp["x"],
                wp["y"],
                "go",
                markersize=8,
                label="waypoint"
            )

        # -------------------
        # GOAL
        # -------------------
        if "goal" in entry:

            goal_x = entry["goal"]["x"]
            goal_y = entry["goal"]["y"]

            ax.plot(goal_x, goal_y, "rx", markersize=12, label="goal")

        # -------------------
        # LIDAR
        # -------------------
        if "ranges" in entry:

            ranges = entry["ranges"]
            angles = entry["angles"]

            for r, a in zip(ranges, angles):

                lx = x + r * math.cos(a)
                ly = y + r * math.sin(a)

                ax.plot([x, lx], [y, ly], "r-", alpha=0.1)

        ax.set_aspect("equal")
        ax.set_title("Robot Navigation Replay")

        ax.legend(loc="upper right")

        plt.pause(0.01)


plt.ioff()
plt.show()