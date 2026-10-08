"""GPS translation anchor + continuous simulated wheel odometry, yaw from odometry."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from .common import load_config, stamp_seconds, spin_node
from .core import gps_to_xy


class Localization(Node):
    def __init__(self):
        super().__init__("gps_localization")
        self.config = load_config(self)
        self.odom = None
        self.offset = None
        self.gps_stamp = -math.inf
        self.pose_pub = self.create_publisher(PoseStamped, "/localization/pose", 10)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix, "/gps/fix", self.on_gps, qos_profile_sensor_data
        )

    def on_gps(self, msg):
        if (
            self.odom is None
            or not math.isfinite(msg.latitude)
            or not math.isfinite(msg.longitude)
        ):
            return
        x, y = gps_to_xy(msg.latitude, msg.longitude, self.config["datum"])
        p = self.odom.pose.pose.position
        self.offset = (x - p.x, y - p.y)
        self.gps_stamp = stamp_seconds(msg.header.stamp)

    def on_odom(self, msg):
        self.odom = msg
        if (
            self.offset is None
            or stamp_seconds(msg.header.stamp) - self.gps_stamp > 0.8
        ):
            return
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
        ) = self.offset
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = msg.pose.pose.position.x + self.offset[0]
        pose.pose.position.y = msg.pose.pose.position.y + self.offset[1]
        pose.pose.position.z = msg.pose.pose.position.z
        pose.pose.orientation = msg.pose.pose.orientation
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    spin_node(node)
