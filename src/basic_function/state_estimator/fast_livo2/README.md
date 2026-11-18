# FAST-LIVO2 integration

The active implementation is the locked native ROS 2 Humble source in
`ros2_upstream/`, with AeroHub-owned ROS 2 adapters and AS2 plugin beside it.
ROS 1 wrappers and Proxies are not part of the supported runtime.

Build, runtime, topic, and acceptance instructions are maintained in
[`deployment/fast_livo2/README.md`](../../../../deployment/fast_livo2/README.md).
The delivered component image contains the algorithm, dependencies, adapters,
plugin, and configuration; it does not mount this source tree or a host colcon
workspace at runtime.
