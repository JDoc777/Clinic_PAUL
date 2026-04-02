import math

def waypoint_step(waypoints, index, pose, tol=0.15):
    #print("waypoint_step called")
    """Return updated index after checking distance to the current waypoint."""
    if index >= len(waypoints):
        return index  # all waypoints done
    
    gx, gy, gtheta = waypoints[index]
    px = pose["x"]
    py = pose["y"]

    dx = gx - px
    dy = gy - py
    dist = math.hypot(dx, dy)

    #print(f"[Waypoint {index}] dist={dist:.3f}")

    if dist < tol:
        #print(f"Reached waypoint {index} → moving to next")
        return index + 2

    return index
