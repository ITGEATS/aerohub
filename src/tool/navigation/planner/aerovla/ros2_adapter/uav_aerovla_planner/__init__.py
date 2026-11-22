"""AeroVLA board-side Planner (Tool layer).

Pure, ROS-free policy and prompt logic live in :mod:`policy` and
:mod:`prompts` so they can be unit-tested on a host without ROS,
transformers or a GPU. The ROS 2 node (:mod:`planner_node`) is the only
module that imports ``rclpy``.
"""
