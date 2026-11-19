# WestStreet 时间同步与运行资源

人物路线仍由原有 LevelSequence 资产定义。`AerohubSceneControl` 为其安装
`FMovieSceneTimeController`，按原生 ProjectAirSim `SimClock` 推进，
不再以 UE 渲染帧的 `DeltaSeconds` 推进。起步 smoothstep 的积分也按仿真时间计算，
不依赖帧率；停播、续播和复位仍由现有 Scene RPC 控制。重复 start 不重启渐变。
仿真暂停时人物保持不动；时钟回退时重新建立时间基准，不倒播或补跑负时间。
重置人物仍需显式 reset，不由原生 scene 重载隐式触发。

飞行基线保持 steppable 3 ms / 3 ms 与 PX4 lockstep。它不保证每秒墙钟都能
推进一秒仿真，但现在人物与飞行仿真使用同一时间基准。相机、推理和 ROS
调度仍可能受录屏负载影响，时钟修复不自动保证完整追踪任务成功。

## 增量编译

使用已有 UE 工具链与已构建的 ProjectAirSim 插件：

```bash
python3 simulator/projectairsim/weststreet/build_scene_control.py \
  --ue-root "$UE_ROOT" --project "$WESTSTREET_PROJECT" \
  --output "$PWD/log/weststreet-plugin-build" --install
```

安装前须关闭该工程的 UE 进程。脚本备份原场景插件，只同步当前插件源码、
二进制与匹配的 BuildId，不替换其他插件或引擎。当前实际入口是
`UnrealEditor <uproject> <map> -game`；此步骤重编译其加载的模块，
不生成独立 Cook/Stage 包。制作独立程序时仍需另行 Build/Cook/Stage 并运行验收。

Linux Editor 模块须直接导入已构建的 `libUnrealEditor-ProjectAirSim.so`。
普通模块依赖会把其公共 `libcore_sim.a` 再链接进场景插件，形成第二份
SimClock 单例。构建脚本以 `nm -D` 检查 `SimClock::Get` 必须是 `U`（导入），
不是 `T`（本模块定义）。游戏单体目标使用正常模块依赖。
其他平台的 Editor 动态库桥接暂未支持；不得复制单例实现绕过检查。

## 纹理预算

24 GiB GPU 的 WestStreet 配置采用 4096 MiB 纹理池并限制到实际显存，
保持 texture streaming 开启。通过项目配置应用，不修改全局引擎配置：

```bash
python3 simulator/projectairsim/weststreet/configure_streaming.py \
  --project "$WESTSTREET_PROJECT" --pool-mib 4096 \
  --backup "$PWD/log/weststreet-config/DefaultEngine-before.ini"
```

其他 GPU 必须重新测量预算。游戏控制台 `DumpTextureStreamingStats` 可读到
Target/Pool，沿实际观察路线检查峰值；初始编辑器视角不能代表运行时需求。
不得通过关闭屏幕提示或设置无限纹理池宣称解决预算不足。

## Landscape 物理材质

打开目标地图，以真实渲染的编辑器运行 `rebuild_landscape.py`。
环境变量 `WESTSTREET_REPAIR_OUTPUT` 指定结果目录。脚本调用原生
`BuildPhysicalMaterial`，显式保存目标地图，结果记录对象名、前后过期计数和保存结果。
重载地图及游戏窗口仍须复核；不以 `-nullrhi` 代替 GPU 构建。

UE 5.2.1 的 `ULandscapeComponent::PostLoad` 仅在 `GIsEditor` 时初始化
临时字段 `LastSavedPhysicalMaterialHash`。因此 `UnrealEditor -game`
仍可能把已保存组件误报为未保存。场景插件针对 UE 5.2，在游戏世界初始化时，
仅对干净 package、有碰撞组件且已存构建哈希与当前材质哈希一致的组件
补齐此临时字段。不重写构建哈希，不修改材质/碰撞数据，也不禁用告警。
真正的哈希不匹配、待处理渲染任务仍由原生检查报告。
此兼容逻辑不用于 Cook 后程序，也不套用于其他引擎版本。
Scene 状态 RPC 暴露 `landscape_materials_outdated`（Editor 构建）和补齐数量供核查。

## 真实场景回归

`validate_sim_clock.py` 必须连接专用测试实例；它会替换 native scene 并暂停仿真，
不得用于活动飞行。使用匹配的 ProjectAirSim Python 环境：

```bash
PYTHONPATH="$PWD" "$PROJECTAIRSIM_PYTHON" \
  simulator/projectairsim/weststreet/validate_sim_clock.py \
  --scene "$TEST_SCENE" --output "$PWD/log/weststreet-clock/result.json"
```

测试从独立 `GetSimTime` 读取原生时间，对照人物时间；覆盖 3 ms / 3 ms、
3 ms / 9 ms、固定仿真时长下的实际位置一致性、仿真暂停、恢复、人物停止、
复位和重复 start。固定 3 秒仿真与 1 秒起步渐变对应 2.5 秒序列进度。
不能假定路线起步段已经达到 5 m/s：资产自身也包含加速，应比较相同序列相位
下的实际位置，而不是将其平均位移硬判为恒速。

GNOME 录屏负载对照必须保持 D-Bus 调用方连接存活直到 StopScreencast；
一次性 `gdbus call` 退出会让录屏提前终止。需核查停止确认和成片时长，
不能仅凭启动返回 true 判定整个测试期间均在录屏。

本次证据与飞行结果见 `log/weststreet-clock-fix/`；各项通过范围以结果文件为准。
