# Downtown West 跑步人物

当前 PX4 + PC 模拟算力板 + AS2 的运行入口见
[WestStreet 框架验证](../../../deployment/airsim/demos/weststreet/README.md)。
本目录的 `launch.py`、`observe.py`、`flight_smoke.py` 保留为早期 SimpleFlight 预览/诊断，
不作为框架飞行验收入口。以下保留资产准备及历史验证记录。

`prepare_runner.py` 在 UE 5.2 中读取原始 `Demo_Environment`，另存
`/Game/Aerohub/WestStreetRunner`，添加一个 Nathan 骨骼人物与自动循环播放的
`/Game/Aerohub/WestRunnerMotion` Level Sequence。原始示例地图不覆盖。

人物沿 UE 坐标 `(375, -4000, -15)` 至 `(375, 4000, -15)` 往返。
单程 80 米、16 秒，平移速度 500 cm/s（5 m/s），整圈 32 秒。
端点瞬间转向；这是可重复的直线运动目标，不是自主行人导航，也没有扫掠避障。
人物带碰撞配置不等于其 Sequencer 运动会自动避开其他物体。

动作使用已传入的 `Sprint_Fwd_Rifle`，原速播放；该动作具有持枪姿态，
人物不附带武器模型。位移速度独立于动画步频，不能据此声称脚底无滑动。
人物 Skeleton 增加动画 Skeleton 的兼容声明；应在编辑器中实际检查姿态。

## 准备

外部工程需已包含 Downtown West、Scanned3DPeoplePack 和 AnimStarterPack。
先设置 `UE_ROOT`、`WEST_PROJECT`、`WEST_RUNNER_OUTPUT` 为本机路径；输出目录使用
仓库忽略的 `log/`。编辑器会修改外部工程并写缓存，先确认文件系统授权。

```bash
"$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd" "$WEST_PROJECT" \
  -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities,SequencerScripting \
  -run=pythonscript -script="$PWD/simulator/projectairsim/weststreet/prepare_runner.py" \
  -unattended -nullrhi -nosound
```

脚本拒绝覆盖已有同名序列，重复执行前先审查已创建资产。

## 可视及运动检查

```bash
"$UE_ROOT/Engine/Binaries/Linux/UnrealEditor" "$WEST_PROJECT" \
  -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities,SequencerScripting \
  -ExecCmds="py $PWD/simulator/projectairsim/weststreet/review_runner.py" \
  -nosound -noraytracing
```

脚本保留编辑器窗口，拍摄道路视图，再启动 Simulate，采样超过 20 秒，
记录实际位置/时间和移动速度，随后结束 Simulate。
结果保存在 `WEST_RUNNER_OUTPUT`，`prepared.json` 只表示地图准备完成。
编辑器验证不等于 Linux 运行包验证，也不代表已接入 ProjectAirSim/PX4。

## 当前验证（2026-11-16）

UE 5.2.1 编辑器 Simulate 实际采样约 20 秒，跨过 16 秒折返点，
直线段速度范围 **4.999957–5.000034 m/s**（容差 0.1 m/s，通过）；
`foot_l` 相对位置随时间变化，确认骨骼动画播放。道路与人物截图已经人工检查。
证据目录：`log/projectairsim/weststreet-runner/`，其中
`motion-validation.json` 为最新速度采样，`runner-street.png` 为道路视图。

原始实现另存地图后序列引用没有跟随，出现“动画播放但位移为零”。
修复为在 LevelSequenceActor 上用 `set_binding` 显式绑定当前地图人物；
UE 5.2 的 `MovieSceneObjectBindingID` 使用无参构造后设置 `guid` 属性。
截图回调有重入风险，检查脚本使用 busy 标记并提前切换阶段。

尚未验证完整 32 秒周期的循环边界、动画脚底无滑动、整段路径碰撞避障，
也未构建 Linux 独立运行包或接入 ProjectAirSim。端点瞬间反向不是自然转身。

## 街区环线

`prepare_route.py` 在现有编辑器中从直线地图创建独立的
`/Game/Aerohub/WestStreetCircuit` 与 `/Game/Aerohub/WestCircuitMotion`。
控制台执行 `py <prepare_route.py 的绝对路径>`；保留已有直线地图与序列。
若同名环线序列已存在，脚本拒绝覆盖。

路线从主街起步，在北侧路口转入横街，沿东侧街道进入广场，
穿行花坛、长椅等物体之间，最后回到主路；约 449.49 米、89.90 秒一圈。
平移按三维路径弧长设置 500 cm/s，转角使用经碰撞复核的二次曲线过渡。
运动由预先生成的 Level Sequence 驱动，不是实时行人避障或 NavMesh AI。

路径准备使用实际场景 Visibility 通道复杂碰撞查询：地面向下射线，
半径 35 cm、半高 80 cm 的人体近似胶囊体扫掠；受阻路段使用 1 米网格 A*，
最后以不超过 50 cm 的采样间距重新检查整条路线。它仅保证有相应碰撞的静态
几何通道可通行，不覆盖未配置碰撞的装饰、运行时移动障碍或全身摆臂包络。

UE 5.2 Python 的命中结果为 `HitResult` 或 `None`，用 `HitResult.to_tuple()`
读取命中、ImpactPoint 等，不能当作 `(bool, HitResult)` 解包。

`review_route.py` 在同一编辑器中启动 Simulate，采样一整圈及下一圈 5 秒，
检查速度、相对于预期路径的位置误差，拍摄路口、广场入口、障碍段及返回段。
`circuit-prepared.json` 仅表示创建成功，实际运行结论看
`circuit-validation.json`；`passed` 不代表已完成独立运行包验证。

准备脚本成功后会保留本地请求回调；向 `WEST_RUNNER_OUTPUT/editor-request.json`
写入 `{"id":"唯一请求号","script":"本仓库检查脚本的绝对路径"}` 可在当前编辑器
执行检查，避免反复启动实例。该文件仅供受信任的本地操作者使用，不对外开放。

### 环线实际验证结果（2026-11-16）

已用真实 UE Simulate 执行完整一圈及下一圈前段；第二次采样 94.91 秒，
实际位置与按序列时间计算的预期路线最大偏差约 15.94 cm，观察到循环回起点。
全程世界时间差分的平均速度约 4.992 m/s。
**严格的逐采样恒速门槛未通过**：转弯处弦长测速低于弧长速度，且直线段也存在
位置与时间采样不一致的波动；不得将 `circuit-validation.json` 的 `passed:false`
描述成严格恒速验收通过。完整保留原始采样和之前失败轮次，尚需进一步定位
低帧率后台运行时的序列求值/采样时序。未构建独立运行包。

另添加 `West_Circuit_FollowCamera`，通过 `add_follow_camera.py` 保存到环线地图，
跟随人物并由 Camera Cut 选择；其偏移为 `(350,-1000,850)` cm。固定相机跟随
不包含相机碰撞避障，靠近棚顶或建筑时可能遮挡；按需切换自由视角。

`capture_route.py` 使用独立 SceneCapture2D 导出四个关键位置的静态编辑器视图，
仅作为环境/放置检查，不是连续运动证据。当前场景阴影较深、棚架遮挡明显，
这些图像不足以声称全程视觉无穿模；几何依据仍为先前的胶囊体碰撞查询。

这是场景功能编辑，不是外部算法复现任务；本轮未新增复现记录或修改正式 Skill。

跟随镜头短运行验证通过：真实游戏世界中相机父对象为跑步人物，PlayerController
的实际 ViewTarget 为该相机；结果见 `follow-camera-validation.json`。

### Play 视角修正（2026-11-16）

用户反馈 Play 视角不对。实测相机虽已附着人物，但镜头方向与人物方向点积为
-0.858，人物位于相机后方；此前仅验证 ViewTarget 和父子关系不足以证明取景正确。
已用 `reset_follow_camera.py` 将跟拍偏移更新为 `(160,-600,320)` cm，朝向人物
前方 170 cm、高度 100 cm 的目标，FOV 75 度；同时退出 Pilot 模式，编辑器自由
浏览不再直接修改跟拍相机。原先 `(350,-1000,850)` 的说明是旧配置。

实际点击当前窗口 Play 验证：PlayerCameraManager 使用该相机位置/朝向，人物
投影落在游戏画面内；截图 `log/projectairsim/weststreet-runner/play-camera-fixed.png`
人工检查通过，人物全身位于画面中下部、前方道路可见。修改只针对观察相机，
不改变运动路线或先前严格恒速验证尚未通过的状态。

## ProjectAirSim + 原有无人机 HUD 融合预览（2026-11-16）

用户确认 HUD 显示正常。本轮仅做场景与插件融合，**不是无人机控制、飞行或算法验收**。
HUD 的速度、峰值、状态和计时仍只属于无人机，不将跑步人物登记为车辆。
Global 是独立观察视角；本街区显式配置为复制已确认的行人跟拍相机。
Tab 在已登记的无人机相机间切换，G 在 Global 与车辆视角间切换。
Global 沿用原规则隐藏无人机速度表，切到 Drone1 可见其真实静止速度 0 m/s。

### 不打包启动

准备好 `UE_ROOT`、`WEST_PROJECT`、`PROJECTAIRSIM_SOURCE`、`PROJECTAIRSIM_PYTHON`
这四个本机变量后，在仓库根目录执行（GUI 会话需已有 DISPLAY/XAUTHORITY）：

```bash
python3 simulator/projectairsim/weststreet/launch.py \
  --ue-root "$UE_ROOT" --project "$WEST_PROJECT" \
  --client-python "$PROJECTAIRSIM_PYTHON" --native-source "$PROJECTAIRSIM_SOURCE" \
  --output "$PWD/log/projectairsim/weststreet-integration/preview"
```

启动器运行现有 `UnrealEditor <工程> /Game/Aerohub/WestStreetCircuit -game`，
不执行 BuildCookRun/Cook/Stage，不要求独立游戏运行包，也不额外打开编辑器窗口。
它显式设置已安装引擎插件的 Linux 动态库搜索路径，避免移动工程后预编译
ProjectAirSim 模块无法解析 ChaosVehicles 等依赖；不修改引擎或原 Blocks 工程。
客户端虚拟环境路径保留符号链接，不能 resolve 为系统 Python 后启动。

默认 topic/service/HUD 端口为 **19889 / 19890 / 19891**，均可用 CLI 修改。
端口已被监听时拒绝启动；已有的其他场景不被停止。结束本启动器（Ctrl+C）或
关闭预览窗口时，仅回收本启动器创建的客户端与游戏进程。
再次执行会显式加载机器人场景；不要对正在飞行的实例使用此入口。

`observe.py` 使用已安装上游配置生成本次 `scene_weststreet.jsonc` 和
`robot_weststreet.jsonc`，仅加载一架 SimpleFlight Drone1，NED 起点 `(8,-45,-0.3)`。
没有 EnableApiControl/Arm/Takeoff/运动命令；原生物理使其落到路面。
只读取真实相机与地面真值并转发车辆遥测，RPC 仅管理观察显示。
这不是 PX4/AS2 执行链。

### 已执行的融合检查

- ProjectAirSim、Drone 内容与 AerohubDisplay 安装到街区，使用同一 UE 5.2.1 BuildId；
  HUD 插件编译成功。原有 Blocks 不覆盖。
- 移除该行人序列的 Camera Cut；保留人物位移动画和循环，由 HUD 统一管理主视角。
- 真实原生状态：API control=false、landed=0、稳定速度 0 m/s；无控制调用。
- 真实 FrontCamera RGB/Depth 与 DownCamera RGB 连续返回，单帧保存于日志目录。
- 编辑器采样验证只有一架 Drone1，Global 的观察相机跟随实际场景相机；
  `editor-validation.json` 通过。最终未烘焙游戏窗口也实际显示 Global/FollowCamera。
- 实际切换 Drone1/Chase，仪表盘显示无人机真值 0 m/s，用户确认 HUD 正常；
  收尾恢复 Global 人物跟随。

证据目录 `log/projectairsim/weststreet-integration/`，最终运行目录为 `preview/`；
其中 `session/observation.json` 是滚动只读观测，`drone-hud.png`、`global-hud.png`
为实际窗口截图。旧失败尝试保留，不作通过依据。

### 尚存边界

- 未烘焙预览仍显示源场景的 `LANDSCAPE PHYSICAL MATERIAL NEEDS TO BE REBUILT`
  提示；没有通过隐藏屏幕消息掩盖。首次运行还会编译 shader。
- 上游街区存在 wind_normal 外部资源引用告警，未在本轮修改资产来源。
- 行人转弯速度采样波动、镜头靠近棚架可能遮挡等原有边界继续保留。
- 独立运行包、PX4、飞行控制及算法闭环均未执行。

## 人物启停与短程飞行验证（2026-11-16）

此前交付是 **HUD/场景插件编译产物 + 未烘焙 UE 预览**，不是独立场景运行包。
融合预览已按用户要求停止。后续增加独立 `AerohubSceneControl` 插件：
人物 **默认暂停**，等待无人机起飞后由调用方启动，不自动绑定无人机状态。
人物控制独立于 HUD，HUD 仍只表示无人机状态。

### 调用人物 RPC

启动 `launch.py` 后，topic/service/HUD/scene-control 默认端口分别为
19889 / 19890 / 19891 / **19892**；最后一个可用 `--scene-port` 修改。

```bash
python3 simulator/projectairsim/weststreet/actor_control.py state
python3 simulator/projectairsim/weststreet/actor_control.py start
python3 simulator/projectairsim/weststreet/actor_control.py stop
python3 simulator/projectairsim/weststreet/actor_control.py reset
```

或直接调用，无需 ProjectAirSim SDK：

```bash
curl --noproxy '*' http://127.0.0.1:19892/aerohub/scene/rpc \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"ControlActor","params":{"actor_id":"runner","event":"start"}}'
```

`stop` 保持位置/动作/序列时间，`start` 从停止点续跑，`reset` 回起点并保持暂停。
未调用 start 时不会跑；调用 start 后仍按原有环线无限循环。
插件构建、标签要求及完整接口见 [AerohubSceneControl](../unreal/AerohubSceneControl/README.md)。

### 已授权的动作联调

```bash
"$PROJECTAIRSIM_PYTHON" simulator/projectairsim/weststreet/flight_smoke.py \
  --execute-flight --motion-mode velocity \
  --output "$PWD/log/projectairsim/weststreet-flight/新建且不存在的运行目录"
```

**此命令真的会在仿真中解锁和飞行**，不是只读展示；仅用于当前 SceneWestStreet/
Drone1/SimpleFlight。先确认新启动场景、无人机在路面静止、无其他控制者。
最大前飞 5 米、水平速度不超过 1 m/s；起飞目标 3 米。预声明空间包络和通过
条件写入每轮 `test-policy.json`，过程中的真实位置/速度和命令保存于 `flight.json`。
使用 SDK `connect_services()`，不抢占观察客户端拥有的单消费者 topic 连接。

流程：人物暂停 → 解锁起飞 → 真值确认高度 → 人物 start → 前飞 → 悬停 → 降落
→ 按真实接地证据解除武装/释放 API → 确认最终 landed → 人物 stop。
失败时先尝试悬停/降落；不能因客户端超时直接判定落地，也不能在未确认接地时
解除武装。结束前核查 final_landed_state=0、final_api_control_enabled=false。

### 结果与限制

- 人物真实 RPC 矩阵通过：`actor-rpc-validation.json`。
- 第一轮 `native_path` **失败**：原生 Takeoff 返回 false，最高约 1.3 米，人物未启动。
  恢复客户端原先 3 秒接收超时不足以完成 Land；后续延长超时，真实路面高度和
  静止样本确认后安全解除控制。失败原始日志和恢复证据完整保留。
- 本次读取的 SimpleFlight 源码默认 `MC_TKO_Z=-2.0`，与 Python Takeoff 注释 3 米
  不同；Takeoff/MoveToPosition 使用同一 MoveOnPathInternal 实现。路径式接口失败
  根因尚未完全确认，**未修复、未修改上游**，不能写成这条原生接口已通过。
- 第二轮 `velocity` 动作矩阵通过：使用原生短时 MoveByVelocity/MoveByVelocityZ
  指令做项目侧反馈协调（高频控制仍由 SimpleFlight 内部执行），避开路径式接口。
  实际起飞 **2.943 米**后才启动人物；前飞目标误差 **0.147 米**；
  悬停位移约 **0.090 米**、末段最大速度约 **0.036 m/s**；Land 返回 true，
  实际高度回到路面且速度为零；最终已解除武装、API=false、landed=0、人物暂停。
- SimpleFlight landed 标志在解除武装前可能仍为 Flying，即使 Land=true 且物理接地。
  本适配先用 Land 返回、相对起始路面高度和速度确认接地，再解除武装，并要求
  最终 landed=0。没有强改 landed 状态，也不将该标志的局限隐去。
- 这是 **SimpleFlight 场景/插件功能联调**，不是目标 PX4/AS2 行为执行链、算法复现
  或真机飞行验收；不替代将来在该架构下的独立适用测试矩阵。

证据：`log/projectairsim/weststreet-flight/flight-attempt-01/`（失败及恢复）、
`flight-attempt-02/`（动作矩阵通过）。原有严格恒速采样和场景资产告警边界不变。
