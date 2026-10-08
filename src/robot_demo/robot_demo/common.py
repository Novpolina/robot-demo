import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def load_config(node):
    node.declare_parameter("scenario", "")
    with open(node.get_parameter("scenario").value, encoding="utf-8") as f:
        return yaml.safe_load(f)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def spin_node(node, on_exit=None):
    """SIGINT can arrive between spin callbacks; suppress only errors from an already closed context."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        if on_exit is not None:
            on_exit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
