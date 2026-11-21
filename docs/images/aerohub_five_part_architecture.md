# 五部分目标架构图

更新日期：2026-11-18。

图稿为 AeroHub 的目标设计，包含已有能力及拟议流程；不据此宣称 Auto Search、通用 Offline / Online Agent、所有 Tool、排行榜或 Gallery 自动化均已实现。Training 明确保留为构思中的部分。PC 与开发板是按任务选择的适配目标，各自的验证状态独立记录。

术语补充（2026-11-21）：Tracking 仅观测，飞行目标跟随归 Following。现有图稿尚未单列 Following，其职责和允许调用 Tracking 的关系以架构正文为准；本次未重绘图稿。

- 位图：[3840 × 2160 PNG](aerohub_five_part_architecture.png)
- 可编辑图稿：[SVG](aerohub_five_part_architecture.svg)
- 绘图源：[Python 渲染脚本](render_five_part_architecture.py)
- 架构正文：[项目简介](../项目简介.md#architecture-design)

## 图中关系

1. **Offline Adaptation**：Auto Search Agent 持续抓取并上架无人机工作。人类在作品商店点击 Download，或 Auto Search Agent 根据效果、会议 / 期刊和已适配基线发现更优工作，均调用 Offline Agent。Agent 使用适配 Skills 完成适配与验证，生成指定平台的适配包及功能卡片；适配经验反馈到 Skills。
2. **Online Runtime**：保留 Application、Mission / Online Agent、Tool、Basic Behavior、基础能力及平台的职责。Navigation、Object Detection、Exploration、Tracking 各附功能卡片图标。状态、地图、传感器及推理接口独立于运动执行通路；控制器和 Behavior 也可直接消费状态。State Estimator 与 Perception 的 A / B 选择器表示通过兼容接口选择实现，不表示无条件热切换。状态输出保持单一权威来源；环境表示不匹配时显式转换或拒绝。
3. **Benchmark**：在明确的任务和平台条件下比较已适配的同类型工作，以前端排行榜展示指标。结果反馈给 Auto Search Agent，作为后续候选筛选依据，不直接切换运行中的控制或定位实现。
4. **Training**：仅提出未来方向；目标、数据和训练流程尚待定义，不虚构已确定的依赖。
5. **Gallery**：面向已适配组件，自动按工作与平台归档并展示运行效果。缩略图为界面示意，展示视频、轨迹、地图、定位和检测 / 跟踪等内容，不是实际验收截图。

黑色实线表示调用或交付；橙色表示数据访问；蓝色虚线表示状态与执行反馈；紫色表示 Skills 的指导与经验更新；绿色虚线表示评测结果回流。

## 绘制与导出

本图遵循手绘工作流图的构图规范：五个编号分区、彩色虚线圆角边框、浅色斜线底纹、黑色描边图标、手写风格标题、明确区分的主流程和反馈线。

本次环境未提供内置图像生成工具，因此采用确定性矢量绘图并导出 PNG，没有调用图像生成 API。文字在 SVG 中保留为可编辑文本。PNG 已固化字体；在其他环境编辑 SVG 时，建议安装 Chilanka 与 Noto Sans CJK SC，避免字体替换造成排版变化。

在具有 Python 3、PyGObject、librsvg 和 pycairo 的环境中执行：

~~~bash
python3 docs/images/render_five_part_architecture.py
~~~

脚本只重建同目录下本图的 SVG 与 PNG；不修改原有的 model_hub_architecture.png，不访问网络。此次渲染使用 16:9、3840 × 2160 画布。

## 构图规格 / Prompt

下列规格保存内容与视觉意图，供后续重绘使用；它不是调用过图像生成模型的记录。

~~~text
Use case: infographic-diagram
Asset type: academic architecture workflow figure
Primary request: Redraw the AeroHub UAV ecosystem using five numbered
panels, dashed rounded borders, light diagonal hatch backgrounds,
small black-outline pictograms, thick main-flow arrows and handwritten headings.

Canvas: 3840 x 2160, 16:9 landscape, white background.
Layout: a large Offline Adaptation panel on the left, a large Online Runtime
panel in the middle, and Benchmark, Training and Gallery stacked on the right.
Use blue, warm orange, green, purple and terracotta pastel panel accents.
Title: "AeroHub / UAV Ecosystem".
Mark the whole figure as "TARGET ARCHITECTURE".

(1) Offline Adaptation:
Auto Search Agent continuously discovers UAV research and updates a Work Store.
Human-triggered route: Work Store Download -> Offline Agent.
Automatic route: compare performance, publication venue and existing adapted
work; a better candidate -> Offline Agent.
Adaptation Skills -> Offline Agent; adaptation experience -> Skills.
Deliver adapted packages and capability cards for PC or development board.
Do not imply PC validation also validates the board.

(2) Online Runtime:
Application / Human mission -> Mission / Online Agent -> Tools.
Tools: Navigation, Object Detection, Exploration, Tracking.
Place a small capability-card pictogram beside EVERY tool.
Motion execution: Basic Behaviors -> Motion Controller -> Platform / PX4.
Basic Behaviors include Takeoff, Land, GoTo, FollowPath (+ FollowTimedPath)
and FollowReference; the timed entry is an extension, not a serial stage.
Basic Functions: State Estimator, Perception, Inference, Motion Controller.
Show A/B selection switches for State Estimator and Perception.
State must have one authoritative output; perception providers need compatible
representations or explicit conversion.
Shared state, maps, sensors and inference interfaces reach Tools directly.
State also reaches Behaviors and the Controller.
Include Sensors / Data Bridges, ROS 2 communication and independent emergency handling.
Platforms include AirSim + PX4 SITL, physical UAVs and other AS2 platforms.
Show execution feedback returning to Mission.

(3) Benchmark:
Evaluate adapted works of the SAME TYPE under comparable task/platform conditions.
Benchmark -> Leaderboard UI. Feed evaluation results back to Auto Search Agent.

(4) Training:
"Concept stage / 构思中"; objectives, data and training flow remain undefined.
No invented implementation or dependency arrows.

(5) Gallery:
Frontend automatically showcases already-adapted works on their target platforms.
Organize videos, trajectories, maps, localization and detection/tracking results
by work and platform. Make illustrative UI thumbnails distinguishable from evidence.

Arrows: black for calls/delivery; orange for shared data access; dashed blue for
state/execution feedback; purple for skill guidance and experience update;
dashed green for benchmark feedback to automatic candidate selection.

Constraints: readable short labels, compact academic density, no shadows,
gradients, 3D, photorealism, external logos, made-up scores or claims of completion.
~~~
