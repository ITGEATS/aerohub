# 视觉链分段耗时打点（2026-11-17）

本轮只加诊断，不调算法/帧率/阈值。证据：
`log/weststreet-demo/live-timing-02/{timing-events,timing-report}.json`及flight-bag。
实际ProjectAirSim/PX4起飞、视觉目标、云台、GPU与Tracking链运行，失跟后正常
System降落上锁并清理。未通过5秒追踪。第一轮因诊断代码遗漏导入导致Tools未就绪，
只采到前段，保留live-timing-01，不将其算作完整链路证据。

## 时钟与边界

所有事件含原始RGB采样stamp、mono_ns、wall_ns、ROS now、节点/PID和事件seq。
同一PC容器共享Linux单调时钟，处理耗时由mono_ns计算；采样间隔仍为仿真时间。
不把仿真时间减墙钟，也不把图像stamp间隔称为推理耗时。

最早打点在PythonSensorAdapter的原生ROS图像回调入口，**不是UE渲染/曝光回调**。
入口之前仍包括原生相机、NNG/C++桥与首次ROS接收，未继续细分，不能声称已测出
“相机采集延迟”。输出ROS发布后缺下游回调，亦只能定位到传输/接收/调度区间，
不能直接断言UDP丢包。

## 本轮匹配耗时

Tracking会话内匹配8帧，以下为中位数，单位ms；不是未收到帧的延迟：

| 边界 | 中位耗时 |
| --- | ---: |
| 原生ROS图像回调→pose_queue（复制/图像转换等） | 40.76 |
| pose_queue→桥发布（姿态配对等待等） | 0.95 |
| 桥发布→检测器回调 | 1.38 |
| 检测器回调→buffer入队 | 0.25 |
| buffer入队→取出 | 0.99 |
| 取出→模型请求前（含诊断保存与准备） | 2.3左右 |
| 模型服务请求→响应（含ROS/HTTP/编码，不是纯GPU核） | 62.07 |
| 测距/TF阶段 | 1.09 |
| 结果发布→Tracking回调 | 1.77 |
| Tracking回调→消费 | 6.57 |
| Tracking处理 | 0.91 |
| 原生ROS图像回调→Tracking处理完成 | 122.48 |

全轮2356个打点，记录范围内三个节点事件seq无缺失。全轮有252ms测距/TF长尾和
185.6ms buffer等待，但都在Tracking会话外；225.5ms Tracking等待对应会话前缓存帧，
随后按session规则拒绝。不能把这些启动/缓存长尾当作本次追踪中的耗时。

## 失跟前的图像去向

最后接纳观测采样时间35.166s，有效到35.666s；反馈35.685s时已lost。

- 35.238s、35.457s两帧：桥接收并发布，之后没有检测器回调事件。
- 35.529s：检测器收到、模型响应、测距及Tracking处理均发生，但预滤波记录
  jump_rejected，未刷新最后有效观测。
- 35.601s：记录camera_transition_or_unknown隔离丢弃。

全轮最大推理输入采样间隔585ms，对应实际buffer取出事件间隔600.2ms；该间隔内
5帧已由桥发布但无检测器回调，是另一个明确的区间证据，不是整张图推理600ms。
这一最大间隔不直接等同首次失跟区间，审计单独保存first_loss时间线。

**结论：这次不是单一模型耗时问题。已发现桥→检测回调的帧缺失区间，以及
过滤拒绝和云台隔离共同造成有效观测中断；入口以前的缺口仍待细分。**

## 使用

诊断默认关闭：检测/Tracking设`timing_enabled:=true`，传感器桥环境
`AEROHUB_VISION_TIMING=1`。事件只含元数据，发送到`/diagnostics/vision_pipeline`，
不额外采集图像、不改source stamp。联合入口已把该topic加入录包。
停止录制后在现有ROS容器运行`audit_timing.py <运行目录>`，生成同stamp阶段关联、
采样/单调事件间隔、拒绝原因、序号完整性、会话内耗时和首次失跟时间线。
只对同宿主有效；跨物理设备需另外测量时钟偏差。记录边界/重复事件不能当丢包。

## 原生发布端对照（live-native-audit-02）

加入单peer NNG透明转发探针：不调用GetImages，不增加相机，不改变payload。
原生Topic使用Pair0，不能与既有C++桥并列连接第二个SDK订阅者；此前并列探针
0帧是方法无效，不能当作采集0Hz。转发端等真实桥首个订阅后才连接引擎，避免
抢占manager场景加载客户端。原生发布记录不等于UE内部曝光线程打点。

同一源采样窗口23.691–46.548s，计数使用唯一stamp：

| 边界 | 帧数 | 实收墙钟Hz | 源采样Hz | 最大源间隔ms |
| --- | ---: | ---: | ---: | ---: |
| 原生Topic转发入口 | 314 | 13.25 | 13.69 | 84 |
| 传感器ROS回调 | 189 | 7.96 | 8.23 | 432 |
| 传感器ROS发布 | 188 | 7.92 | 8.18 | 432 |
| 检测器回调 | 101 | 4.33 | 4.47 | 1101 |
| buffer消费 | 101 | 4.33 | 4.47 | 1101 |

原生→传感器回调缺125帧；传感器发布→检测回调缺87帧；云台隔离1帧。
检测回调→buffer消费没有进一步缺帧。本轮原生发布稳定但并非精确15Hz；秒级
间隙出现在其后。不能把某次局部666ms间隙称为全程1.5Hz，也不能用成功收到帧的
低延迟解释未到达帧。探针转发耗时中位0.738ms、最大4.598ms。

证据：`log/weststreet-demo/live-native-audit-02/native-rate-audit.json`、
`native-frames.jsonl`与`flight-bag/`。原任务未通过，已确认降落解除武装并停止所属服务。
该轮UDP探针遗漏Python自身进程（扫描瞬时FD消失导致整进程被跳过），不能据其
零drop认定没有UDP丢包；已修正逐FD异常处理，并用临时localhost socket核对
SO_RCVBUF读取正确，下一轮独立验证传输缺口原因。

### 修正socket探针的复核（live-native-audit-03）

共同源窗口22.824–47.856s：原生344帧/13.25墙钟Hz，ROS回调233帧/8.97Hz，
ROS发布229帧/8.81Hz，检测109帧/4.25Hz，buffer106帧/4.13Hz。
原生最大84ms，检测最大1164ms。4帧云台隔离，3帧检测接收后未进入消费。

- Python传感器桥pid29，UDP56663，SO_RCVBUF=425984，drop635→1300（+665）。
- 算力板Tool进程pid7，UDP56685，SO_RCVBUF=425984，drop319→948（+629）。
- C++原生桥所见socket没有drop增加。

这些drop是UDP datagram而不是完整图像数；不能与缺失RGB帧一一换算。
DDS profile请求4MiB，而host rmem_max=212992，Linux加倍记账后实际socket只有416KiB；
一张RGB原始像素921600字节，分片突发期间存在接收溢出证据。仅比较单帧尺寸和
缓冲不能证明必丢（取决于并行排空）；因此进一步单变量提高rmem_max做真实A/B。

离线审计入口：`python3 deployment/airsim/demos/weststreet/audit_transport.py <运行目录>`。
它只读停止录制后的CDR String诊断与native JSONL，不需要额外采集。

### 单变量缓冲上限A/B结果（live-native-audit-05）

保持相机、推理、滤波和任务参数不变，仅临时将host rmem_max从212992提高到4194304。
实际DDS socket SO_RCVBUF由425984变为8388608（Linux加倍记账），两个接收进程的
所有已观测socket drop增量均为0。结束后已恢复rmem_max=212992，未写持久配置。

| 边界（墙钟Hz） | 基线03 | 接收缓冲对照05 |
| --- | ---: | ---: |
| 原生发布入口 | 13.25 | 13.12 |
| 传感器ROS回调 | 8.97 | 13.12 |
| 传感器ROS发布 | 8.81 | 12.64 |
| 检测器回调 | 4.25 | 12.64 |
| buffer消费 | 4.13 | 12.27 |

05共同源窗口21.966–49.803s（精确窗口以native-rate-audit.json为准），380帧原生
全部到达传感器回调；其中14帧云台切换隔离，余366帧全部到达检测器；11帧因最新帧
消费策略未进入buffer_take。原生最大墙钟间隔96.62ms；检测230.85ms；buffer326.61ms。
两段传输缺帧从111/120降到0/0，支持**两段DDS/UDP接收溢出是主要掉频原因**。
缓冲小于单帧不是充分条件；本结论结合实际drop和单变量A/B，而非仅凭尺寸推断。

05的5秒追踪任务返回成功，独立离线录包门全部通过：19个不同采集目标更新，
追踪窗实际水平位移19.44m，248条执行轨迹参考，原生Tool降落与解除武装确认。
仅为有界阶段通过，完整环线、可靠性和完整故障矩阵仍未验收。
证据：该轮native-rate-audit.json、evaluation.json、mission/result.json、cleanup.json，
以及目录外live-native-audit-05-host-buffer-ab.json（系统参数恢复记录）。
