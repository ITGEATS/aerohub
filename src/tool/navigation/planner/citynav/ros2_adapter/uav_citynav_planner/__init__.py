"""uav_citynav_planner：CityNavAgent HSPM + 在线记忆语言导航 Planner（迁移方案阶段 5）。

依赖闭包 = rclpy + numpy/scipy + stdlib（onboard-computer v0.1.1 容器 py3.10
实测无 networkx，底层图用包内 UndirectedGraph，D24；
禁 torch/cv2/sklearn——grounding/CLIP 走 Inference HTTP 服务，统计滤波用
scipy cKDTree 等义重写，见 perception.py）。
"""
