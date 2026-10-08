#!/usr/bin/env python3
"""Generate checked-in SDF from the single scenario configuration."""
from pathlib import Path
import yaml

root = Path(__file__).resolve().parents[1] / "src/robot_demo"
config = yaml.safe_load((root / "config/scenario.yaml").read_text())


def box(name, x, y, sx, sy, sz, color, collision=True):
    geometry = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    return f"""<model name="{name}"><static>true</static><pose>{x} {y} {sz/2} 0 0 0</pose><link name="link">
    {f'<collision name="collision">{geometry}</collision>' if collision else ''}
    <visual name="visual">{geometry}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
    </link></model>"""


parts = [
    """<?xml version="1.0"?><sdf version="1.8"><world name="demo">
<physics name="physics" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
<gravity>0 0 -9.8</gravity>
<plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
<plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
<plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
<plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<plugin filename="libignition-gazebo-navsat-system.so" name="ignition::gazebo::systems::NavSat"/>
<plugin filename="libignition-gazebo-contact-system.so" name="ignition::gazebo::systems::Contact"/>
<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
<latitude_deg>55.0</latitude_deg><longitude_deg>37.0</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates>
<scene><ambient>0.7 0.7 0.7 1</ambient><background>0.12 0.15 0.20 1</background><shadows>true</shadows></scene>
<light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 -0.3 -1</direction></light>
<model name="ground"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry></collision>
<visual name="visual"><geometry><plane><normal>0 0 1</normal><size>24 16</size></plane></geometry><material><ambient>0.65 0.69 0.73 1</ambient><diffuse>0.65 0.69 0.73 1</diffuse></material></visual></link></model>
"""
]
for o in config["obstacles"]:
    color = "1 0.45 0.08 1" if o.get("sensor_only") else "0.22 0.28 0.36 1"
    parts.append(box(o["name"], o["x"], o["y"], o["sx"], o["sy"], o["sz"], color))
parts.append(box("START_A", 0, 0, 0.9, 0.9, 0.002, "0.1 0.75 0.4 1", False))
parts.append(box("FINISH_B", 8, 0, 0.9, 0.9, 0.002, "0.1 0.45 1 1", False))
parts.append(
    """<gui fullscreen="false"><plugin filename="MinimalScene" name="3D View"><gz-gui><title>Gazebo - GPS navigation A to B</title><property type="string" key="state">docked</property></gz-gui><engine>ogre2</engine><scene>scene</scene><ambient_light>0.7 0.7 0.7</ambient_light><background_color>0.12 0.15 0.20</background_color><camera_pose>4 -8 10 0 0.85 1.5708</camera_pose></plugin><plugin filename="GzSceneManager" name="Scene Manager"/><plugin filename="InteractiveViewControl" name="Interactive view control"/><plugin filename="WorldControl" name="World control"><play_pause>true</play_pause><start_paused>false</start_paused></plugin><plugin filename="WorldStats" name="World statistics"/></gui>
</world></sdf>"""
)
(root / "worlds").mkdir(exist_ok=True)
(root / "worlds/demo.sdf").write_text("\n".join(line.rstrip() for line in "\n".join(parts).splitlines()) + "\n")
print(root / "worlds/demo.sdf")
