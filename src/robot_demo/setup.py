from setuptools import setup
from glob import glob

setup(
    name="robot_demo",
    version="1.0.0",
    packages=["robot_demo"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robot_demo"]),
        ("share/robot_demo", ["package.xml"]),
        *[
            (f"share/robot_demo/{d}", glob(f"{d}/*"))
            for d in ("launch", "config", "urdf", "worlds")
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="maintainer@example.com",
    description="Verified simulated GPS navigation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "localization = robot_demo.localization:main",
            "perception = robot_demo.perception:main",
            "navigator = robot_demo.navigator:main",
            "validator = robot_demo.validator:main",
        ]
    },
)
