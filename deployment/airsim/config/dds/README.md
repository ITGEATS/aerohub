# DDS 配置

`fastdds_udp.xml` 是公共 UDPv4 transport 配置，由 PC 镜像和部署入口引用。
`tools/robopi/robopi.sh` 根据 `.env` 的 PC_IP 与 ROBOPI_IP 生成两端接口白名单，
写入 `log/airsim/robopi-runtime/`，不修改这里的公共模板。

两端 ROS_DOMAIN_ID 由运行配置选择；HDC 连接与 DDS 网络连通性分别检查。

## 图像传输的宿主接收缓冲上限

**DDS XML 请求的缓冲大小不等于 socket 实际获得的大小。** 使用 UDP 传输图像时，
把宿主接收上限和实际 socket 缓冲纳入部署检查；不能只检查相机 Hz 和模型耗时。
公共模板请求 `receiveBufferSize=4194304`（4 MiB），仍受 Linux
`net.core.rmem_max` 限制。host 网络容器共享宿主网络命名空间，应在宿主检查。

WestStreet 的 640×480 RGB 每帧原始像素为921600字节（900 KiB）。真实对照发现：

| 项目 | 原上限 | 仅提高接收上限后 |
| --- | ---: | ---: |
| `net.core.rmem_max` | 212992（208 KiB） | 4194304（4 MiB） |
| DDS socket 实际 `SO_RCVBUF` | 425984（416 KiB） | 8388608（8 MiB） |
| 原生图像发布，墙钟 Hz | 13.25 | 13.12 |
| 检测器接收，墙钟 Hz | 4.25 | 12.64 |
| 两个 Python 接收端 UDP drop 增量 | 665 / 629 | 0 / 0 |

提高上限后，原生发布→桥接收、桥放行→检测器的缺帧均归零。
这说明图像分片突发可在模型处理之前耗尽接收缓冲；加采集线程或优化GPU推理
不能修复这类传输丢帧。**缓冲小于一帧并不自动意味着必丢**，还取决于分片和排空
速度；本结论依据实际socket drop和单变量对照，不把4 MiB视作所有负载的通用保证。

### 部署检查与临时配置

在实际接收端所在网络命名空间检查，启动 DDS 进程前设置测试上限：

```bash
sysctl net.core.rmem_max
# 先保存原值；以下命令需要管理员权限，仅修改运行时配置。
sudo sysctl -w net.core.rmem_max=4194304
```

随后重启相关接收进程，让新socket重新申请缓冲；仅修改上限不会自动扩容已有socket。
用 `ss -uamnp` 的 `skmem`（rb接收上限、d丢包）或接收进程内的
`getsockopt(SO_RCVBUF)` 核对实际值，并观察逐socket/inode的drop增量。
Linux会对 `SO_RCVBUF` 加倍记账，显示8 MiB不等于XML误配，也不是启动时立刻占满8 MiB。

上限影响该网络命名空间中后续socket的申请权限，不会自动给所有socket分配该容量。
多无人机、多进程需评估累计内存；持续消费不足时，更大缓冲可能增加旧帧积压，
因此同时检查帧龄、最大间隔，并保留应用层有界最新帧策略。测试结束恢复原值；
长期配置由部署维护者显式管理，本目录模板本身不会修改宿主sysctl。

上述对照结束已恢复208 KiB，**不代表后续启动自动具备4 MiB上限**。
WestStreet联合入口会在启动图像链前检查上述4 MiB上限前提，不足时拒绝启动，
但仍需用实际socket检查确认DDS配置生效。
真实对照范围、逐阶段帧率与有界任务结果见
[WestStreet诊断记录](../../demos/weststreet/TIMING.md)。
