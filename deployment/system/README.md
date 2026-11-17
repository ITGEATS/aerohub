# System Tool 部署

服务在算力侧，任务 Demo 在工作站。固定共享镜像不变，以独立overlay补编译。
当前实现和验证边界见 [验证记录](VALIDATION.md)，接口语义见
[源码说明](../../src/tool/system/README.md)。

```bash
bash deployment/system/build.sh
bash deployment/system/test.sh
```

默认工作区是被忽略的 `log/system-tool/workspace`，两个脚本支持首个参数指定
新的独立工作区。不要指向其他任务共享的构建目录。

算力容器挂载本仓库与构建工作区，并给同车所有运动Tool挂载同一个持久化目录。
例如将宿主专用目录挂到 `/motion`，设置 `AEROHUB_MOTION_LOCK_DIR=/motion`。
以下命令须在已经运行的相应角色容器内执行；不负责创建或接管已有容器：

```bash
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /work/install/setup.bash
export AEROHUB_MOTION_LOCK_DIR=/motion
UAV_NAMESPACE=drone0 USE_SIM_TIME=true bash /repo/deployment/system/start.sh
```

此处 `/work` 是 System overlay 挂载位置。AS2、PX4、传感器和唯一 `/clock` 必须
已就绪。同一ROS_DOMAIN_ID和车辆命名空间，System Tool不启动模型或仿真器。
后续与SUPER联用时，重编译并加载包含 `uav_motion_resource` 的SUPER overlay；
`deployment/airsim/demos/super_forest/build_navigation_overlay.sh` 已补入该依赖。
System与SUPER必须使用同一持久化motion挂载及环境变量。

工作站加载ROS和同一System接口/client overlay后：

```bash
source /opt/ros/humble/setup.bash
source /work/install/setup.bash
python3 /repo/deployment/system/demo.py --output /evidence/system-demo
```

`demo.py` 只调用 System Tool：查询就绪 → 起飞3m → 降落 → 查询未解锁和资源释放。
不使用仿真RPC、Behavior Action、平台服务或运动参考。该命令会请求真实起降，
当前尚未完成本版仿真验收；不要将示例命令视为已执行记录。

如用DroneInterface加载Module，设置：

```bash
export AS2_MODULES_PATH=/work/install/uav_system_python_api/share/uav_system_python_api/modules
```

合并安装布局对应 `/work/install/share/uav_system_python_api/modules`。
后续扩展状态、系统操作时保持业务请求/查询接口；每个新增能力明确前置条件、
副作用、完成证据、取消与失败处理。不会默认把所有平台底层服务透传给任务层。
