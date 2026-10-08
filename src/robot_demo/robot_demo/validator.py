"""Independent runtime acceptance monitor. A successful controller status alone is insufficient."""
import csv
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Image, NavSatFix, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from .common import LATCHED, load_config, stamp_seconds, spin_node
from .core import rectangle_distance, cell, gps_to_xy


class Validator(Node):
    def __init__(self):
        super().__init__("acceptance_monitor")
        self.config = load_config(self)
        self.declare_parameter("report_dir", "artifacts")
        self.declare_parameter("timeout", 180.0)
        self.output = Path(self.get_parameter("report_dir").value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        self.counts = {}
        self.stamps = {}
        self.valid_depth = 0
        self.valid_scan = 0
        self.contact_events = 0
        self.contacts_seen = 0
        self.samples = []
        self.truth_samples = []
        self.maximum_tilt = 0.0
        self.low_mapped = False
        self.low_depth_seen = False
        self.command = Twist()
        self.zero_since = None
        self.success = None
        self.done = False
        self.gps = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(
            TFMessage, "/ground_truth/poses", self.on_truth, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/lidar/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/depth/image", self.on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/depth/obstacles", self.on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix, "/gps/fix", self.on_gps, qos_profile_sensor_data
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, LATCHED)
        self.create_subscription(
            RosPath, "/plan", lambda m: self.note("plan", m), LATCHED
        )
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(
            Contacts, "/contacts", self.on_contacts, qos_profile_sensor_data
        )
        self.create_subscription(String, "/mission/status", self.on_status, LATCHED)
        self.trajectory_pub = self.create_publisher(RosPath, "/trajectory", LATCHED)
        self.create_timer(0.2, self.tick)

    def note(self, name, msg):
        self.counts[name] = self.counts.get(name, 0) + 1
        if hasattr(msg, "header"):
            self.stamps.setdefault(name, []).append(stamp_seconds(msg.header.stamp))

    def on_pose(self, msg):
        self.note("pose", msg)
        p = msg.pose.position
        self.samples.append((stamp_seconds(msg.header.stamp), p.x, p.y))
        if len(self.samples) % 5 == 0:
            path = RosPath()
            path.header = msg.header
            for t, x, y in self.samples[::5]:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.trajectory_pub.publish(path)

    def on_truth(self, msg):
        self.counts["truth"] = self.counts.get("truth", 0) + 1
        for tr in msg.transforms:
            if tr.child_frame_id in ("demo_robot", "demo_robot::base_link"):
                q = tr.transform.rotation
                # Angle between chassis +Z and world +Z, independent of yaw.
                cosine = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                tilt = math.acos(max(-1.0, min(1.0, cosine)))
                self.maximum_tilt = max(self.maximum_tilt, tilt)
                p = tr.transform.translation
                self.truth_samples.append(
                    (self.get_clock().now().nanoseconds * 1e-9, p.x, p.y)
                )
                break

    def on_scan(self, msg):
        self.note("lidar", msg)
        if any(
            math.isfinite(r) and msg.range_min < r < msg.range_max for r in msg.ranges
        ):
            self.valid_scan += 1

    def on_depth(self, msg):
        self.note("depth", msg)
        if msg.encoding == "32FC1":
            arr = np.ndarray(
                (msg.height, msg.width),
                dtype=">f4" if msg.is_bigendian else "<f4",
                buffer=bytes(msg.data),
                strides=(msg.step, 4),
            )
            if np.any(np.isfinite(arr) & (arr > 0.1) & (arr < 8)):
                self.valid_depth += 1

    def on_gps(self, msg):
        self.note("gps", msg)
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.gps = gps_to_xy(msg.latitude, msg.longitude, self.config["datum"])

    def on_cloud(self, msg):
        if self.low_depth_seen:
            return
        low = next(o for o in self.config["obstacles"] if o.get("sensor_only"))
        for x, y, z in read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if (
                abs(x - low["x"]) < low["sx"] / 2 + 0.08
                and abs(y - low["y"]) < low["sy"] / 2 + 0.08
                and 0.04 < z < low["sz"] + 0.06
            ):
                self.low_depth_seen = True
                break

    def on_map(self, msg):
        self.note("map", msg)
        low = next(o for o in self.config["obstacles"] if o.get("sensor_only"))
        spec = self.config["map"]
        grid = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        a = cell(low["x"] - low["sx"] / 2, low["y"] - low["sy"] / 2, spec)
        b = cell(low["x"] + low["sx"] / 2, low["y"] + low["sy"] / 2, spec)
        self.low_mapped |= bool(np.any(grid[a[1] : b[1] + 1, a[0] : b[0] + 1] >= 50))

    def on_cmd(self, msg):
        self.command = msg
        stopped = abs(msg.linear.x) < 1e-6 and abs(msg.angular.z) < 1e-6
        if stopped:
            if self.zero_since is None:
                self.zero_since = time.monotonic()
        else:
            self.zero_since = None

    def on_contacts(self, msg):
        self.contacts_seen += 1
        for contact in msg.contacts:
            names = str(contact)
            if any(o["name"] in names for o in self.config["obstacles"]):
                self.contact_events += 1

    def on_status(self, msg):
        status = json.loads(msg.data)
        if status["state"] == "SUCCEEDED" and self.success is None:
            self.success = time.monotonic()

    def tick(self):
        if self.done:
            return
        if self.success and time.monotonic() - self.success > 2.0:
            self.finish("mission_completed")
        elif time.monotonic() - self.started > self.get_parameter("timeout").value:
            self.finish("timeout")

    def finish(self, reason):
        self.done = True
        trajectory = self.truth_samples
        length = sum(
            math.dist(a[1:], b[1:]) for a, b in zip(trajectory, trajectory[1:])
        )
        minimum = min(
            (
                rectangle_distance(x, y, o) - self.config["robot"]["radius"]
                for _, x, y in trajectory
                for o in self.config["obstacles"]
            ),
            default=-math.inf,
        )
        final_error = (
            math.dist(trajectory[-1][1:], self.config["finish"])
            if trajectory
            else math.inf
        )
        start_error = (
            math.dist(trajectory[0][1:], self.config["start"])
            if trajectory
            else math.inf
        )
        tf_ok = rclpy.ok() and all(
            self.tf.can_transform("map", frame, rclpy.time.Time())
            for frame in ("base_link", "lidar_link", "camera_optical_frame", "gps_link")
        )
        stopped_motion = bool(
            trajectory
            and len(trajectory) > 30
            and math.dist(trajectory[-1][1:], trajectory[-20][1:]) < 0.03
        )
        gps_error = (
            math.dist(self.gps, self.config["finish"])
            if self.gps is not None
            else math.inf
        )
        checks = {
            "mission_succeeded": self.success is not None,
            "starts_in_A": start_error < 0.20,
            "finishes_in_B_ground_truth": final_error < 0.25,
            "finishes_in_B_GPS": gps_error < 0.25,
            "no_obstacle_contacts": self.contact_events == 0,
            "positive_ground_truth_clearance": minimum > 0.02,
            "detour_not_straight_line": length > 8.3,
            "lidar_has_real_measurements": self.valid_scan > 30,
            "depth_has_real_measurements": self.valid_depth > 30,
            "GPS_messages_received": self.counts.get("gps", 0) > 30,
            "depth_only_obstacle_added_to_map": self.low_mapped,
            "depth_only_obstacle_measured_by_camera": self.low_depth_seen,
            "map_and_plan_published": self.counts.get("map", 0) > 5
            and self.counts.get("plan", 0) > 0,
            "TF_chain_complete": tf_ok,
            "zero_velocity_after_arrival": self.zero_since is not None
            and time.monotonic() - self.zero_since > 1.0,
            "robot_actually_stopped": stopped_motion,
            "ground_truth_received": len(trajectory) > 100,
            "chassis_stays_level": self.maximum_tilt < 0.06,
        }
        report = {
            "passed": all(checks.values()),
            "reason": reason,
            "checks": checks,
            "metrics": {
                "ground_truth_final_error_m": final_error,
                "gps_final_error_m": gps_error,
                "minimum_conservative_clearance_m": minimum,
                "travelled_m": length,
                "start_error_m": start_error,
                "maximum_chassis_tilt_deg": math.degrees(self.maximum_tilt),
                "obstacle_contact_events": self.contact_events,
                "contact_messages": self.contacts_seen,
                "message_counts": self.counts,
                "wall_duration_s": time.monotonic() - self.started,
                "sensor_rates_hz": {
                    name: (len(stamps) - 1) / (stamps[-1] - stamps[0])
                    for name, stamps in self.stamps.items()
                    if len(stamps) > 1 and stamps[-1] > stamps[0]
                },
            },
        }
        (self.output / "acceptance.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
        for name, points in [
            ("trajectory.csv", self.samples),
            ("ground_truth.csv", trajectory),
        ]:
            with (self.output / name).open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["sim_time_s", "x_m", "y_m"])
                writer.writerows(points)
        self.get_logger().info(
            f'ACCEPTANCE {"PASS" if report["passed"] else "FAIL"}: {json.dumps(checks)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Validator()
    spin_node(node, lambda: node.finish("interrupted") if not node.done else None)
