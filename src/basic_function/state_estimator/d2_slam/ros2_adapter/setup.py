from setuptools import find_packages, setup


package_name = "uav_d2_slam_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="AeroHub maintainers",
    maintainer_email="maintainers@example.com",
    description="ROS 2 side of the D2SLAM sensor and result bridge.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": ["d2_slam_bridge = uav_d2_slam_ros2.node:main"],
    },
)
