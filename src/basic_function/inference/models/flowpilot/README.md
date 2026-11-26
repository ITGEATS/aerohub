# FlowPilot CUDA 模型

上游 [FlowPilot](https://github.com/flyingbitac/FlowPilot) 固定提交 `f29f1bfee1399c96e5fd384276d5d7a09876286b`，源码作为子模块保存在 `src/tool/navigation/planner/flowpilot/upstream`。模型模块只引用推理实现，不执行运动。四份模型文件来源/revision 见 [prepare.py](prepare.py)，文件名和 SHA256 由 [weights.sha256](weights.sha256) 唯一维护。上游为 Apache-2.0，第三方权重保留各自许可说明。

## 开发准备

Linux x86_64、CUDA 兼容驱动、支持当前 lock 的 uv 和固定干净上游为前置。模型使用 Python 3.11、torch 2.5.1+cu124 独立环境，ROS 使用 Humble 自己的环境。

```bash
git submodule update --init src/tool/navigation/planner/flowpilot/upstream
bash src/basic_function/inference/models/flowpilot/setup.sh --check-cuda
```

准备只处理权重、锁定环境、加载/预热和一次额外前向，不启动服务。权重默认放本模型忽略的 artifacts；已有正确文件复用，版本或散列不符则失败。不要复制 Windows venv 到 Linux。开发服务使用模型自己的 Python；正式容器按 [交付镜像](../../../../../deployment/inference/flowpilot/README.md) 构建，不挂宿主环境。

## 输入输出

ROS `InferFlowPilot` 接收轴向深度和同观测时刻的 25 维状态：机体系 P/V/A/J、机体到世界 wxyz 四元数、目标 P/V/A。ROS 边界支持32FC1米/16UC1毫米，处理端序/行跨度；整帧无有效值拒绝，稀疏坏点填20m，再裁剪/归一化并resize为96×160。

生产 HTTP 用 base64(zlib(小端float32米制图像)) 与shape传输；兼容JSON深度数组。输出48×12 P/V/A/J样本与8×3 Bernstein控制点，30Hz末样本为47/30s，名义域1.6s。服务保留request_id与observation_header。具体接口见 [InferFlowPilot.srv](../../interfaces/srv/InferFlowPilot.srv)，后处理与停止属于Planner。

开发进程可用 `FLOWPILOT_WEIGHTS_DIR` 指定模型数据目录，始终执行同一SHA256清单校验。交付镜像将这四份数据编入/models，不挂宿主权重；该环境变量在镜像内指向已编入的位置。无CUDA时明确失败，不静默降级CPU。

## 数值复核与状态

```bash
src/basic_function/inference/models/flowpilot/.venv/bin/python deployment/airsim/demos/flowpilot_single/check_parity.py --cuda
```

脚本固定输入和种子，对照全部样本与控制点；与上游一致不证明Planner后处理避障安全。CPU、GPU、ROS和真实仿真分别记账。新dev适配的实际通过项和未覆盖项见 [验证记录](../../../../../docs/flowpilot-validation.md)，不能借历史服务器结果宣称新镜像或公共接口通过。
