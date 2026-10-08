"""Fuse known static map with live lidar/depth hits; publish the planning map and filtered cloud."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from .common import LATCHED, load_config, stamp_seconds, spin_node
from .core import initial_map, cell, depth_points, yaw_of, above_floor


class Perception(Node):
    def __init__(self):
        super().__init__("sensor_mapping")
        self.config = load_config(self)
        self.spec = self.config["map"]
        self.grid = initial_map(self.config)
        self.pose = None
        self.lidar_hits = set()
        self.depth_hits = set()
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", LATCHED)
        self.scan_pub = self.create_publisher(
            LaserScan, "/lidar/scan", qos_profile_sensor_data
        )
        self.depth_pub = self.create_publisher(
            Image, "/depth/image", qos_profile_sensor_data
        )
        self.rgb_pub = self.create_publisher(
            Image, "/rgb/image", qos_profile_sensor_data
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/depth/obstacles", qos_profile_sensor_data
        )
        self.stats_pub = self.create_publisher(String, "/perception/stats", 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/camera/depth_image", self.on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/camera/image", self.on_rgb, qos_profile_sensor_data
        )
        self.create_timer(0.5, self.publish_map)

    def on_pose(self, msg):
        self.pose = msg

    def on_rgb(self, msg):
        msg.header.frame_id = "camera_optical_frame"
        self.rgb_pub.publish(msg)

    def add_points(self, points, source):
        if self.pose is None:
            return
        p = self.pose.pose.position
        yaw = yaw_of(self.pose.pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        mapped = np.column_stack(
            (
                p.x + c * points[:, 0] - s * points[:, 1],
                p.y + s * points[:, 0] + c * points[:, 1],
                points[:, 2],
            )
        )
        for x, y, z in mapped:
            ix, iy = cell(x, y, self.spec)
            if 0 <= ix < self.spec["width"] and 0 <= iy < self.spec["height"]:
                self.grid[iy, ix] = 100
                source.add((ix, iy))
        return mapped

    def pose_is_fresh(self, stamp):
        return (
            self.pose is not None
            and abs(stamp_seconds(stamp) - stamp_seconds(self.pose.header.stamp)) < 0.3
        )

    def on_scan(self, msg):
        msg.header.frame_id = "lidar_link"
        self.scan_pub.publish(msg)
        if not self.pose_is_fresh(msg.header.stamp):
            return
        ranges = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges > 0.35) & (ranges < msg.range_max - 0.05)
        points = np.column_stack(
            (
                ranges[valid] * np.cos(angles[valid]),
                ranges[valid] * np.sin(angles[valid]),
                np.full(np.sum(valid), 0.38),
            )
        )
        self.add_points(points, self.lidar_hits)

    def on_depth(self, msg):
        msg.header.frame_id = "camera_optical_frame"
        self.depth_pub.publish(msg)
        if not self.pose_is_fresh(msg.header.stamp):
            return
        points = depth_points(msg)
        # Camera pose in URDF: x=0.24, z=0.19 relative to base at floor height 0.13.
        points[:, 0] += 0.24
        points[:, 2] += 0.32
        points = points[above_floor(points)]
        mapped = self.add_points(points, self.depth_hits)
        if mapped is not None:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = "map"
            self.cloud_pub.publish(
                create_cloud_xyz32(header, mapped.astype(np.float32))
            )

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.spec["resolution"]
        msg.info.width = self.spec["width"]
        msg.info.height = self.spec["height"]
        msg.info.origin.position.x = self.spec["origin_x"]
        msg.info.origin.position.y = self.spec["origin_y"]
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.flatten().astype(np.int8).tolist()
        self.map_pub.publish(msg)
        self.stats_pub.publish(
            String(
                data=f"lidar_cells={len(self.lidar_hits)} depth_cells={len(self.depth_hits)}"
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    spin_node(node)
