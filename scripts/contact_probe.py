#!/usr/bin/env python3
"""Negative integration check: deliberately overlapping spawn must produce contact messages."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts

rclpy.init()
node = Node("contact_probe")
events = []


def callback(msg):
    for c in msg.contacts:
        if "wall_one" in str(c):
            events.append(str(c))


subscription = node.create_subscription(
    Contacts, "/contacts", callback, qos_profile_sensor_data
)
deadline = time.monotonic() + 20
while not events and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
out = Path("artifacts/contact_probe.json")
out.parent.mkdir(exist_ok=True)
out.write_text(
    json.dumps(
        {
            "passed": bool(events),
            "obstacle_contact_events": len(events),
            "sample": events[:1],
        },
        indent=2,
    )
)
print(out.read_text())
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if events else 1)
