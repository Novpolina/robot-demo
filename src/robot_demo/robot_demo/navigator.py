"""A* on inflated sensor map + short-lookahead tracking with fresh-sensor interlock."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image, NavSatFix
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from .common import LATCHED, load_config, stamp_seconds, spin_node
from .core import astar, inflate, cell, world, gps_to_xy, yaw_of, wrap


class Navigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigation")
        self.config = load_config(self)
        self.spec = self.config["map"]
        self.robot = self.config["robot"]
        self.declare_parameter("start_delay", 4.0)
        self.start_delay = self.get_parameter("start_delay").value
        self.goals = [
            gps_to_xy(g["latitude"], g["longitude"], self.config["datum"])
            for g in self.config["waypoints"]
        ]
        self.pose = None
        self.blocked = None
        self.path = []
        self.index = 0
        self.finished = False
        self.last_seen = {}
        self.last_plan = -100.0
        self.start_wall = time.monotonic()
        self.front_clear = math.inf
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/plan", LATCHED)
        self.status_pub = self.create_publisher(String, "/mission/status", LATCHED)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/mission/markers", LATCHED
        )
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, LATCHED)
        self.create_subscription(
            LaserScan, "/lidar/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Image,
            "/depth/image",
            lambda m: self.touch("depth", m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            "/gps/fix",
            lambda m: self.touch("gps", m),
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self.publish_markers)
        self.report("WAITING")

    def touch(self, name, msg):
        self.last_seen[name] = (time.monotonic(), stamp_seconds(msg.header.stamp))

    def on_pose(self, msg):
        self.pose = msg
        self.touch("pose", msg)

    def on_map(self, msg):
        self.blocked = inflate(
            np.asarray(msg.data, dtype=np.int16).reshape(
                msg.info.height, msg.info.width
            ),
            self.robot["radius"] + self.robot["clearance"],
            self.spec["resolution"],
        )
        self.touch("map", msg)

    def on_scan(self, msg):
        self.touch("lidar", msg)
        points = [
            (
                r * math.cos(msg.angle_min + i * msg.angle_increment),
                r * math.sin(msg.angle_min + i * msg.angle_increment),
            )
            for i, r in enumerate(msg.ranges)
            if math.isfinite(r)
        ]
        self.front_clear = min(
            (x for x, y in points if x > 0 and abs(y) < 0.27), default=math.inf
        )

    def report(self, state, **extra):
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "state": state,
                        "waypoint_index": self.index,
                        "waypoint_count": len(self.goals),
                        **extra,
                    }
                )
            )
        )

    def stop(self):
        self.cmd_pub.publish(Twist())

    def replan(self, x, y, now):
        cells = astar(
            self.blocked,
            cell(x, y, self.spec),
            cell(*self.goals[self.index], self.spec),
        )
        self.path = [world(ix, iy, self.spec) for ix, iy in cells]
        self.last_plan = now
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for px, py in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)
        if not self.path:
            self.report("NO_PATH")

    def tick(self):
        wallnow = time.monotonic()
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.finished:
            self.stop()
            return
        # Both wall and simulation time freshness: a paused/dead simulator cannot leave a moving command latched.
        ready = all(
            k in self.last_seen
            and wallnow - self.last_seen[k][0] < 2.0
            and now - self.last_seen[k][1] < 1.2
            for k in ("lidar", "depth", "gps", "pose", "map")
        )
        if not ready or wallnow - self.start_wall < self.start_delay:
            self.stop()
            return
        x, y = self.pose.pose.position.x, self.pose.pose.position.y
        distance = math.dist((x, y), self.goals[self.index])
        if distance < self.robot["goal_tolerance"]:
            self.index += 1
            self.path = []
            self.stop()
            if self.index == len(self.goals):
                self.finished = True
                self.report("SUCCEEDED", final_error_m=distance)
                self.get_logger().info(
                    f"Mission succeeded; final GPS-based error {distance:.3f} m"
                )
            else:
                self.report("WAYPOINT_REACHED")
            return
        invalid = any(
            self.blocked[cell(px, py, self.spec)[1], cell(px, py, self.spec)[0]]
            for px, py in self.path
        )
        if not self.path or invalid or now - self.last_plan > 2.0:
            self.replan(x, y, now)
        if not self.path:
            self.stop()
            return
        nearest = min(
            range(len(self.path)), key=lambda i: math.dist((x, y), self.path[i])
        )
        target = self.path[-1]
        for point in self.path[nearest:]:
            if math.dist((x, y), point) >= self.robot["lookahead"]:
                target = point
                break
        yaw = yaw_of(self.pose.pose.orientation)
        error = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
        cmd = Twist()
        cmd.angular.z = max(-0.9, min(0.9, 2.2 * error))
        if abs(error) < 0.45 and self.front_clear > 0.46:
            cmd.linear.x = min(
                self.robot["max_speed"], max(0.08, 0.6 * distance)
            ) * max(0.3, math.cos(error))
        if self.front_clear <= 0.46 and abs(error) < 0.15:
            self.stop()
            self.report("SAFETY_STOP")
            return
        self.cmd_pub.publish(cmd)
        self.report(
            "NAVIGATING",
            distance_m=distance,
            front_clear_m=min(99.0, self.front_clear),
            path_points=len(self.path),
        )

    def publish_markers(self):
        array = MarkerArray()
        for i, (label, point, color) in enumerate(
            [
                ("A - START", self.config["start"], (0.1, 1.0, 0.4)),
                ("B - FINISH", self.config["finish"], (0.1, 0.6, 1.0)),
            ]
        ):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "zones"
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = float(point[0])
            m.pose.position.y = float(point[1])
            m.pose.position.z = 0.7
            m.pose.orientation.w = 1.0
            m.scale.z = 0.35
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            m.text = label
            array.markers.append(m)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    spin_node(node, lambda: node.stop() if rclpy.ok() else None)
