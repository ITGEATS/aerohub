# 模型实现与历史案例

## 判断差异发生在哪一层

| 现象/比较 | 核查方向 |
| --- | --- |
| 同一经纬度下无噪声世界磁场不同 | 模型、表版本、插值、单位、地理原点 |
| 世界磁场相同，机体分量随姿态不符 | NED/ENU、FRD/FLU、旋转方向或重复旋转 |
| 模拟器输出正确，PX4 收到的值不符 | MAVLink 序列化、单位、传感器实例、时间和有效位 |
| 实际测量与参考相符，EKF 仍拒绝 | 有效参数、初始化、传感器选择、延迟、偏置与融合条件 |
| 仅地图/显示航向不符 | 状态到地图的注册；另查转换，不能据此认定地磁错误 |

在 AirSim 1.8.1 的原版参考实现中，MagnetometerSimple 默认使用 DipoleModel；特定 PX4 版本使用查表模型。模型不同可能使磁偏角、磁倾角和强度不同，是否超过目标版本检查门限必须实测。其他 AirSim fork 与 PX4 版本重新核查。

## 可复用实现形状

目标 AirLib 的相对入口通常是：

```text
include/sensors/magnetometer/MagnetometerSimpleParams.hpp
include/sensors/magnetometer/MagnetometerSimple.hpp
include/common/Px4WorldMagneticModel.hpp             # 案例新增
include/common/px4_wmm/geo_magnetic_tables.hpp        # 案例引入
include/common/px4_wmm/PX4_LICENSE.txt                # 保留对应许可
```

1. 在实际 PX4 构建输入中找到地磁查询实现及其表。历史入口为 `src/lib/world_magnetic_model/geo_mag_declination.cpp`、`.h` 和 `geo_magnetic_tables.hpp`。核查 CMake/链接来源，不从文件名猜测运行版本。
2. 按锁定提交引入表和对应查询逻辑，保持纬度夹取、经度处理、网格下取整、边界索引、双线性插值及缩放规则。避免重新实现“差不多”的连续模型。保留原版权与 BSD 许可。
3. 在 AirSim 参考磁场分支添加明确模型选择，例如 `ReferenceSource_Px4Wmm`。让当前场景实际选择新分支，保留其他模式供已有场景兼容。
4. 用环境实际经纬度查询，不将某个场景的磁场向量写成全局常量。若目标模型只用经纬度，则按该契约保持一致；不要自行加入高度或当前日期修正后声称与 PX4 完全等价。
5. 从偏角 D、倾角 I、强度 F 重建世界 NED 磁场；角度换成弧度：

```text
H  = F * cos(I)
BN = H * cos(D)
BE = H * sin(D)
BD = F * sin(I)
```

这组公式约定倾角向下为正，必须对照目标 PX4 定义。历史实现用 Gauss：`nT × 1e-9 → Tesla`，`Tesla × 1e4 → Gauss`，因此 `1 Gauss = 1e-4 Tesla`。不要被旧源码中的错误注释带偏；同时核对传输协议要求的单位。

6. 把 `Vector3r(BN, BE, BD)` 交回原磁力计输出流程，维持世界 NED → 机体 FRD 的旋转、比例、噪声、偏置和时间戳。不要在 MAVROS/ROS ENU 转换层再次修正同一磁场。
7. 更新生效配置或默认值并构建插件。若要支持 settings 切换，必须同时实现该字段的解析；新增 JSON 字段本身不会改变 C++ 枚举。历史案例是在源码中改默认值，未提供可直接照抄的模型切换 JSON。

## 历史 RACER / AirSim 案例边界

以下是可追溯的已读实现事实，不是所有平台的推荐版本，也不是此次重新运行的验收结果：

| 项目 | 核查事实 |
| --- | --- |
| 环境 | UE 4.27 场景工程自带的 AirSim 插件 |
| PX4 提交 | `61907962c9220437ec01a98fa26bfbf5cbb254b5` |
| 地磁表 | WMM-2020；AirSim 引入表与本地 PX4 参考表 SHA256 相同 |
| 表 SHA256 | `3e985cba6a1fd9b246d628bf1d24b843b306a5a6d60ec69900188ec535a3b8cb` |
| 查询 | `Px4WorldMagneticModel.hpp`，复制匹配提交的表查询形状与浮点单位换算 |
| 传感器启用 | `MagnetometerSimpleParams` 默认 `ReferenceSource_Px4Wmm` |
| 生成测量 | `MagnetometerSimple::updateReference` 生成 NED 场；原输出函数旋转到机体并加噪声/偏置 |
| PX4 启动脚本 | 设置 `EKF2_DECL_TYPE=3`、`EKF2_MAG_TYPE=0`；这些是历史脚本值，使用前核对目标参数定义和运行时读回 |
| 状态 | 源码分支和表一致性已核查；既往双机飞行有独立报告，此次技能整理未重建插件、未重跑磁场专项验证 |

同一案例中的落地 yaw/原点标定是估计坐标到公共地图的注册，不是地磁模型修复。关闭 EKF 磁融合也不是该案例采用的解决方式。

可独立获取的上游参考：

- [锁定 PX4 查询源码](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/lib/world_magnetic_model/geo_mag_declination.cpp)
- [锁定 PX4 地磁表](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/lib/world_magnetic_model/geo_magnetic_tables.hpp)

更新 PX4、插件或场景包后重新比较实际模型。即使两个版本都写 WMM-2020，也需对照表内容及查询代码；模型名称一致不是数值一致证明。
