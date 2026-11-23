# YOPO-Tracker Following 接口

本入口连接已运行的 YOPO-Tracker 后端。它将后端的 start、stop 和任务状态转换为公共 `FollowObject` Action。YOPO-Tracker 从 RGB-D 模型候选自主选目标，支持 `AUTO=1`，请求中的 `query` 必须为空。Elastic 支持 `QUERY=0`。

先启动现有 YOPO-Tracker 后端，再从仓库根目录构建并启动 Following 接口容器。使用与算力侧 System 和 Navigation 相同的 ROS domain 和宿主运动锁目录：

```bash
docker build -f deployment/yopo_tracker_humble/Dockerfile.following -t uav/yopo-following:humble .
docker run --rm --network host -e ROS_DOMAIN_ID \
  -e AEROHUB_MOTION_LOCK_DIR=/motion \
  -v "${AEROHUB_MOTION_LOCK_DIR:?set the shared host motion directory}:/motion" \
  uav/yopo-following:humble
```

开发环境也可在加载 `uav_planner_interfaces` 和 `uav_motion_resource` overlay 后直接运行脚本。任务侧加载同一版本的接口并发送 Goal：

```bash
ros2 action send_goal /drone0/tools/following/follow \
  uav_planner_interfaces/action/FollowObject \
  '{selection_mode: 1, query: "", timeout: 30.0}' --feedback
```

发送前由 Mission 完成起飞并确认悬停。后端默认地址为 `/yopo_tracker_live`，可用 ROS 参数 `backend` 指定。每辆车部署一个 Following 服务端。任务结束等待 Action 的 `stop_confirmed`，由 Mission 执行降落。

Following 公共入口在接单时获取车辆运动锁，停止确认后释放。直接调用后端 start 属于诊断路径，部署应只让一个托管入口发起运动。锁目录跨容器保留，未确认停止的占用记录按 [System Tool 恢复规则](../../src/tool/system/README.md#运动互斥边界) 处理。
