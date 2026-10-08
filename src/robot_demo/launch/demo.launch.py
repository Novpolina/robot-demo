import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot_demo")
    description = xacro.process_file(
        os.path.join(share, "urdf/robot.urdf.xacro")
    ).toxml()
    params = {
        "use_sim_time": True,
        "scenario": os.path.join(share, "config/scenario.yaml"),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("start_delay", default_value="4.0"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument(
                "report_dir", default_value=os.path.abspath("artifacts/latest")
            ),
            ExecuteProcess(
                cmd=[
                    "ign",
                    "gazebo",
                    "-r",
                    "-s",
                    "-v",
                    "3",
                    os.path.join(share, "worlds/demo.sdf"),
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ign",
                    "gazebo",
                    "-g",
                    "--gui-config",
                    os.path.join(share, "config/gazebo.config"),
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"use_sim_time": True, "robot_description": description}],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world",
                    "demo",
                    "-name",
                    "demo_robot",
                    "-string",
                    description,
                    "-z",
                    "0.13",
                    "-x",
                    LaunchConfiguration("spawn_x"),
                    "-y",
                    LaunchConfiguration("spawn_y"),
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": os.path.join(share, "config/bridge.yaml"),
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_demo",
                executable="localization",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="robot_demo",
                executable="perception",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="robot_demo",
                executable="navigator",
                parameters=[
                    params,
                    {"start_delay": LaunchConfiguration("start_delay")},
                ],
                output="screen",
            ),
            Node(
                package="robot_demo",
                executable="validator",
                parameters=[params, {"report_dir": LaunchConfiguration("report_dir")}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(share, "config/demo.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
