"""rclpy 级集成测试占位（阶段 6/7 在 ROS 2 环境执行，见迁移方案 M2/M3）。

本包测试 venv 仅 stdlib+pytest，不装 rclpy；node.py 的可纯化辅助
（PNG 编码、cameras 解析、取帧/新鲜度判定、上游同构消息构造、缓存与有限
重试）已全部下沉到 http_client.py，并由 test_http_client.py 覆盖。
阶段 6（AirSim+PX4 SITL 闭环）与阶段 7（M2 真实 DDS 生命周期）在此文件补：
  - 五目 sensor_measurements/<Cam>/rgb/image 订阅与最新帧缓存（含 stamp 保留）；
  - citynav/llm/query→result 端到端 JSON 契约与 error_code 枚举
    （ok|stale|timeout|http|parse|busy）；
  - use_sim_time 下帧龄判定；busy 单飞拒答；cache_dir 命中不请求。
"""
