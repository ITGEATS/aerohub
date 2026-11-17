"""node 级 ROS 测试占位（阶段 6/7：真实 DDS 生命周期、五目批次新鲜度、
FollowTimedPath 修订/确认时序、cancel/FAULT 收口注入）。

本包纯逻辑模块（perception/hspm/memory/trajectory/clients）已由同目录其余
pytest 覆盖；node.py 依赖 rclpy 与 uav_planner_interfaces，需在 ROS 2 环境
（onboard-computer 容器）运行，见迁移方案 M2/M6。"""
import pytest

pytest.skip("node 级 ROS 测试在阶段 6/7 的 ROS 2 环境执行", allow_module_level=True)
