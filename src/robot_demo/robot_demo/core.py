"""ROS-independent geometry and navigation algorithms, tested with pytest."""
import heapq
import math
import numpy as np

EARTH_RADIUS = 6378137.0


def gps_to_xy(latitude, longitude, datum):
    """Local ENU approximation, appropriate for this 12 m world, axes E/N."""
    return (
        EARTH_RADIUS
        * math.cos(math.radians(datum["latitude"]))
        * math.radians(longitude - datum["longitude"]),
        EARTH_RADIUS * math.radians(latitude - datum["latitude"]),
    )


def xy_to_gps(x, y, datum):
    return (
        datum["latitude"] + math.degrees(y / EARTH_RADIUS),
        datum["longitude"]
        + math.degrees(x / (EARTH_RADIUS * math.cos(math.radians(datum["latitude"])))),
    )


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def cell(x, y, spec):
    return (
        math.floor((x - spec["origin_x"]) / spec["resolution"]),
        math.floor((y - spec["origin_y"]) / spec["resolution"]),
    )


def world(ix, iy, spec):
    return (
        spec["origin_x"] + (ix + 0.5) * spec["resolution"],
        spec["origin_y"] + (iy + 0.5) * spec["resolution"],
    )


def initial_map(config):
    spec = config["map"]
    grid = np.zeros((spec["height"], spec["width"]), dtype=np.uint8)
    # Known static scene map; the low obstacle is intentionally absent, discovered by depth.
    for obstacle in config["obstacles"]:
        if obstacle.get("sensor_only", False):
            continue
        for iy in range(spec["height"]):
            for ix in range(spec["width"]):
                x, y = world(ix, iy, spec)
                if (
                    abs(x - obstacle["x"]) <= obstacle["sx"] / 2
                    and abs(y - obstacle["y"]) <= obstacle["sy"] / 2
                ):
                    grid[iy, ix] = 100
    return grid


def inflate(grid, metres, resolution):
    blocked = grid >= 50
    result = blocked.copy()
    radius = math.ceil(metres / resolution)
    h, w = grid.shape
    ys, xs = np.nonzero(blocked)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if math.hypot(dx, dy) * resolution > metres:
                continue
            xx, yy = xs + dx, ys + dy
            valid = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
            result[yy[valid], xx[valid]] = True
    # The robot footprint must also remain inside the defined map.
    result[:radius, :] = True
    result[-radius:, :] = True
    result[:, :radius] = True
    result[:, -radius:] = True
    return result


def astar(blocked, start, goal):
    h, w = blocked.shape

    def free(c):
        return 0 <= c[0] < w and 0 <= c[1] < h and not blocked[c[1], c[0]]

    if not free(start) or not free(goal):
        return []
    queue = [(math.dist(start, goal), 0.0, start)]
    costs, parents = {start: 0.0}, {}
    while queue:
        _, cost, here = heapq.heappop(queue)
        if cost > costs.get(here, math.inf):
            continue
        if here == goal:
            path = [here]
            while here in parents:
                here = parents[here]
                path.append(here)
            return path[::-1]
        for dx, dy in (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ):
            nxt = (here[0] + dx, here[1] + dy)
            if not free(nxt):
                continue
            # Prevent passing diagonally through two touching occupied cells.
            if (
                dx
                and dy
                and (
                    not free((here[0] + dx, here[1]))
                    or not free((here[0], here[1] + dy))
                )
            ):
                continue
            newcost = cost + math.hypot(dx, dy)
            if newcost < costs.get(nxt, math.inf):
                costs[nxt] = newcost
                parents[nxt] = here
                heapq.heappush(queue, (newcost + math.dist(nxt, goal), newcost, nxt))
    return []


def depth_points(message, hfov=1.2):
    """Project metric axial depth into camera FLU coordinates; respects encoding and row padding."""
    dtype = {
        "32FC1": np.dtype(">f4" if message.is_bigendian else "<f4"),
        "16UC1": np.dtype(">u2" if message.is_bigendian else "<u2"),
    }.get(message.encoding)
    if dtype is None:
        raise ValueError(f"Unsupported depth encoding: {message.encoding}")
    depth = np.ndarray(
        (message.height, message.width),
        dtype=dtype,
        buffer=bytes(message.data),
        strides=(message.step, dtype.itemsize),
    )[::4, ::4].astype(float)
    if message.encoding == "16UC1":
        depth *= 0.001
    u, v = np.meshgrid(np.arange(0, message.width, 4), np.arange(0, message.height, 4))
    focal = message.width / (2 * math.tan(hfov / 2))
    valid = np.isfinite(depth) & (depth > 0.3) & (depth < 6.0)
    return np.column_stack(
        (
            depth[valid],
            -(u[valid] - (message.width - 1) / 2) * depth[valid] / focal,
            -(v[valid] - (message.height - 1) / 2) * depth[valid] / focal,
        )
    )


def rectangle_distance(x, y, o):
    return math.hypot(
        max(abs(x - o["x"]) - o["sx"] / 2, 0), max(abs(y - o["y"]) - o["sy"] / 2, 0)
    )


def above_floor(points):
    """Reject the floor even with small chassis pitch/height errors.

    RANSAC fits z=a*x+b*y+c, restricted to nearly horizontal planes near
    the expected floor. This cannot mistake a vertical wall for the floor.
    Depth-camera mounting and the flat-world assumption are explicit.
    """
    candidates = points[(points[:, 2] < 0.30) & (points[:, 0] > 0.6)]
    plane = np.array([0.0, 0.0, 0.0])
    best = np.zeros(len(candidates), dtype=bool)
    if len(candidates) >= 30:
        rng = np.random.default_rng(7)
        design = np.column_stack((candidates[:, :2], np.ones(len(candidates))))
        for _ in range(40):
            sample = rng.choice(len(candidates), 3, replace=False)
            if abs(np.linalg.det(design[sample])) < 1e-5:
                continue
            trial = np.linalg.solve(design[sample], candidates[sample, 2])
            if math.hypot(trial[0], trial[1]) > 0.15 or abs(trial[2]) > 0.08:
                continue
            inliers = np.abs(design @ trial - candidates[:, 2]) < 0.015
            if np.count_nonzero(inliers) > np.count_nonzero(best):
                best = inliers
                plane = trial
        if np.count_nonzero(best) >= 30:
            plane = np.linalg.lstsq(design[best], candidates[best, 2], rcond=None)[0]
    height = points[:, 2] - (
        points[:, 0] * plane[0] + points[:, 1] * plane[1] + plane[2]
    )
    return (height > 0.055) & (height < 1.3) & (points[:, 0] > 0.4)
