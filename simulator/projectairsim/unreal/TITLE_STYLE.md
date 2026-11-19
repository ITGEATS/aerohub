# UE5.2 HUD 标题样式调研

## 结论

当前 `AerohubDisplay` 走的是 `AHUD` + `FCanvasTextItem`，适合采用稳定、低依赖的标题样式：**固定粗体字体 + 固定彩色单色填充 + 深色描边**。标题文字仍由 RPC 动态传入，样式不随文本内容变化。

**实施状态（2026-11-15）：用户已确认采用推荐方案。已接入独立 Noto CJK Bold 字体、暖金色填充（sRGB #FFD166）、深色窄描边（#181410），标题字号从基准 28 提高到 36。居左位置、HUD 遮挡顺序和 RPC 协议保持。实际构建与画面检查结果见下方实施验证。**

建议参数方向：

- 字体：新增并随插件分发真正的 CJK Bold 字体文件，例如与现有 Regular 同系列的 `NotoSansCJK-Bold.ttc`；不要用放大、偏移重绘或当前 Regular 文件冒充粗体。
- 填充：保持单一高饱和但不过亮的颜色；Canvas 的文本颜色直接适合该需求。
- 描边：使用深色 `OutlineColor` 和 `bOutlined = true`，优先于仅使用右下阴影。当前 Canvas 实现的描边是四个固定 `(-1,-1)`、`(-1,1)`、`(1,1)`、`(1,-1)` 偏移，因此它是固定窄描边，不是可任意调宽、模糊或多层的艺术描边。
- 阴影：可选。若启用，应只作轻微辅助，不应与深色描边叠成厚重脏边。
- 不建议在本轮为渐变字、纹理字或任意艺术字改造为 UMG；这会改变当前 HUD 的渲染与生命周期边界。

## 当前插件事实

当前实现位于：

- `simulator/projectairsim/unreal/AerohubDisplay/Source/AerohubDisplay/Private/AerohubDisplay.cpp`
- `simulator/projectairsim/unreal/AerohubDisplay/Source/AerohubDisplay/Public/AerohubDisplay.h`
- `simulator/projectairsim/unreal/AerohubDisplay/Resources/Fonts/`

截至 2026-11-15 的工作树中：

1. `BeginPlay()` 动态创建 `UFont`，设置 `EFontCacheType::Runtime`，再把磁盘字体文件加入 `CompositeFont.DefaultTypeface`。等宽 HUD 标签分别使用 Nimbus Mono Regular/Bold；RPC 标题和中心消息使用 `NotoSansCJK-Regular.ttc`。
2. `DrawLabel()` 使用 `FCanvasTextItem` 绘制固定 HUD 标签；`DrawTextBlock()` 使用 `TextFont` 绘制 `DisplayTitle` 和 `CenterMessage`，调研初始为白色填充加黑色阴影。
3. 插件构建规则将 `Resources/Fonts/...` 作为 `NonUFS` 运行时依赖随包分发。
4. 调研初始仓库只有 `NotoSansCJK-Regular.ttc`，`fc-scan` 显示其中包含 JP/KR/SC/TC/HK 及对应 Mono 的 Regular 字面；没有 Bold 字面。文件 SHA-256 为 `b76b0433203017ca80401b2ee0dd69350349871c4b19d504c34dbdd80541690a`。

因此，“标题改为粗体”需要引入实际 Bold 字体资源并让标题字体引用它；仅把已有 Regular 条目标成 `Bold` 名称不会改变字形轮廓。若未来将 Regular 与 Bold 放入同一个 Composite Font，也应分别建立命名 typeface entry，并通过 `FSlateFontInfo::TypefaceFontName` 选择；当前每个文件各建一个 `UFont` 的方式则可直接让标题使用独立的 Bold `UFont`。

## UE5.2 Canvas 当前可做什么

本机 UE 源码版本的直接依据：

- `<ue5_root>/Engine/Source/Runtime/Engine/Public/CanvasItem.h`
- `<ue5_root>/Engine/Source/Runtime/Engine/Private/UserInterface/CanvasItem.cpp`
- `<ue5_root>/Engine/Source/Runtime/Engine/Classes/Engine/Font.h`
- `<ue5_root>/Engine/Source/Runtime/SlateCore/Public/Fonts/SlateFontInfo.h`

### 字体与真正字重

`CanvasItem.h` 的 `FCanvasTextItem` 同时接受 `UFont` 和 `FSlateFontInfo`。`Font.h` 明确说明 Runtime cached 字体由组合字体中的 TTF/OTF 字体文件构成，所需字形在运行时按需缓存；`FSlateFontInfo` 则提供 `TypefaceFontName` 选择组合字体中的字面。因此 Canvas 可以显示真正的 Bold，前提是提供 Bold font face，而不是由 Canvas 自动加粗。

Epic 的 Font Asset 文档也说明 Runtime 字体使用 Composite Font；一个字体家族可包含 Normal、Bold、Italic 等多个 Font Face，运行时按字符范围和字面名称选择最合适的字体。当前插件虽然是在 C++ 中直接构造 `UFont`，但使用的是同一套 Runtime/Composite Font 路径。

### 单色填充

`FCanvasTextItem` 构造函数接收 `FLinearColor`；`FCanvasItem::SetColor()` 也可修改颜色。运行时缓存字体在 `CanvasItem.cpp` 中使用 alpha-only 字形纹理并强制相应混合模式，最终以传入颜色调制字形。因此当前 Canvas 路径天然适合固定单色填充，不提供逐字形内部的渐变参数。

### 阴影

`FCanvasTextItemBase::EnableShadow(Color, Offset)` 支持阴影颜色和二维偏移，默认偏移为 `(1,1)`；零偏移会关闭阴影。实现会把阴影作为一个额外文本效果偏移加入绘制。这是硬边偏移阴影，不是带模糊半径、扩散或多层参数的阴影系统。

### 描边

`FCanvasTextItemBase` 暴露 `bOutlined` 和 `OutlineColor`。本机 `CanvasItem.cpp` 第 909-916 行在启用描边时固定增加四次对角偏移绘制：`(-1,-1)`、`(-1,1)`、`(1,1)`、`(1,-1)`；这些偏移没有乘以 DPI scale，也没有读取 outline width。相比之下，第 903-907 行的阴影偏移会乘以 `DPIScale`。因此 Canvas `bOutlined` 的限制已经由当前本地 cpp 实现确认：它是固定四向偏移的窄描边，没有公开宽度、模糊、扩散、多层或 DPI 缩放参数。它在视觉上是否足够，仍需实现后按目标分辨率实际验收。

需要注意：源码中虽然计算了经过 Canvas alpha 调制的描边颜色局部变量，四个 outline effect 实际加入的是 `OutlineColor` 本身。样式设计应依赖已验证的颜色与透明度结果，不应假定它和填充、阴影具有完全一致的 alpha 调制行为。

## UMG/Slate 才适合的样式

Epic 官方的 UMG Font Materials and Outlines 文档明确区分：没有 Font Material 时是单色文字；指定 UI Domain 的 Font Material 后，可在 shader 中使用 Vertex Color 等输入做颜色运算，并可为 outline 指定独立颜色和 Material。由此可以实现渐变、纹理、动态材质参数等效果。

这些能力不能等同于当前 `FCanvasTextItem` 的 `FLinearColor`、阴影和四向描边：

| 需求 | 当前 Canvas 路径 | UMG/Slate Text + UI Font Material |
| --- | --- | --- |
| 真正 Regular/Bold 字重 | 可以，必须提供对应 font face | 可以，通过 Font Asset/Typeface 选择 |
| 固定单色填充 | 可以 | 可以 |
| 固定偏移硬阴影 | 可以 | 可以 |
| 深色窄描边 | 可以，固定四向 1 px 风格 | 可以，outline size/material 更可控 |
| 字内渐变、纹理、动画材质 | 无直接文本样式接口 | 适合，用 UI Domain Font Material |
| 任意“艺术字”外形 | 仅当它本身是可加载且含目标字形的字体；否则不行 | 字体可配材质，但完全定制字形仍需真实字体、预制图片/图集或专用渲染方案 |

“艺术字体”还必须与动态 RPC 文本区分。预制标题图片可以获得任意视觉造型，却只能覆盖事先制作的字符串；动态中英文 RPC 标题需要字体文件实际包含每个 Unicode 字符的 glyph。材质只能改变已有 glyph 的着色，不能补出字体中不存在的汉字。

## 中文动态 RPC 文本覆盖

当前链路具备显示动态中文的基本条件：

1. HTTP body 通过 `FUTF8ToTCHAR` 从 UTF-8 转为 Unreal `TCHAR`。
2. JSON 字符串进入 `FString DisplayTitle` 或 `FString CenterMessage`。
3. 绘制时通过 `FText::FromString()` 交给 Runtime cached `UFont`。
4. `NotoSansCJK-Regular.ttc` 的本地字体元数据包含简体中文 `Noto Sans CJK SC Regular`，同时也有繁体中文、日本语、韩语字面。Noto 官方说明 Noto CJK 支持简体中文、繁体中文、日文、韩文，并覆盖 Han、Hiragana、Katakana、Hangul、Bopomofo 等脚本。

但应保留以下边界：

- “CJK 覆盖广”不等于覆盖所有 Unicode 字符。罕见扩展汉字、emoji、特殊符号或新版本 Unicode 字符仍可能缺字；RPC 接口允许任意字符串时，不能宣称绝对全覆盖。
- 当前代码把 TTC 文件作为一个默认 typeface entry 加载，没有显式选择 TTC 的 SC face index。提交 Bold 资源前应在目标 UE5.2 运行包中用简体、繁体、日文、韩文和混合 ASCII 的代表字符串验证实际选面与字形区域化结果。
- `DrawTextBlock()` 按 `FString` 的 `TCHAR` 单元逐个 `Mid(I,1)` 换行和截断。BMP 内常用中文通常可用，但非 BMP 字符可能由代理对组成；逐单元截断可能拆开代理对。组合附加符、emoji ZWJ 序列和复杂 grapheme cluster 也可能被拆分。这不妨碍常用中文标题，但属于“任意动态 Unicode 文本”承诺之外的限制。
- 粗体中文必须使用同等 CJK 覆盖的 Bold face。Nimbus Mono Bold 适合现有 ASCII/拉丁 HUD 标签，不应作为中文标题回退字体。

建议在不改协议的前提下，最少验证这些 RPC 文本：

- 简体中文：`城市低空巡检任务`
- 繁体中文：`城市低空巡檢任務`
- 中英数字混排：`Drone 01：返航高度 120 m`
- 常用标点与省略号：`任务准备中……`
- 缺字观察样例：至少加入一个目标业务确实可能发送的扩展汉字或符号，而不是用合成替代结果判断覆盖。

## Noto CJK Bold 许可与仓库资源

现有资源：

- `simulator/projectairsim/unreal/AerohubDisplay/Resources/Fonts/NotoSansCJK-Regular.ttc`
- `simulator/projectairsim/unreal/AerohubDisplay/Resources/Fonts/Noto-LICENSE`

仓库内 `Noto-LICENSE` 将上游标为 `noto-cjk`，Source 指向官方 `notofonts/noto-cjk` 仓库，并对字体文件标注 `SIL-1.1`。随附 OFL 1.1 正文允许字体与软件捆绑、嵌入和再分发，要求保留版权与许可文本，且不得单独销售字体；修改版本还受 Reserved Font Name 条款约束。

Noto 官方文档同样说明所有 Noto 字体使用 Open Font License，可以用于数字产品和商业项目，但不可单独销售字体。官方 Noto CJK 历史明确列出 `NotoSansCJK-Regular.ttc` 和 `NotoSansCJK-Bold.ttc`，说明 Bold TTC 是该系列的正式字重资源。

因此，加入未经修改的官方 `NotoSansCJK-Bold.ttc` 并与现有许可文本一同分发，原则上符合当前仓库采用的 OFL 1.1 资源方式。落地时仍应：

1. 从 Noto 官方发布源取得文件，记录版本、下载来源和 SHA-256，不从不明字体站取包。
2. 核对下载包自身的 `LICENSE` 与当前 `Noto-LICENSE` 一致；若版本或版权声明变化，随资源更新许可文件。
3. 保持原始字体名称，除非确实修改字体；若修改，遵守 Reserved Font Name 和重命名要求。
4. 在打包产物中确认 Bold TTC 与许可文件都被 `RuntimeDependencies` 收入。


## 推荐方案

在当前 Canvas 架构内采用以下方案，风险和改动面最小：

1. 标题专用真正的 Noto Sans CJK Bold 运行时字体，覆盖动态中文、英文和数字。
2. 固定彩色 `FLinearColor` 填充，不做文字内部渐变。
3. 启用深色 `OutlineColor` + `bOutlined`，以复杂仿真画面中的稳定可读性为主。
4. 仅在实机画面验证确有需要时保留轻微阴影；不要用多次偏移模拟可调粗描边。
5. 继续通过现有 RPC 只传文字内容，不把颜色、字体名或材质路径开放成任意远端参数，以维持统一视觉和可控资源集合。
6. 若以后有明确需求做渐变、纹理、发光或动画标题，再单独评估 UMG/Slate Text Block + UI Domain Font Material；不要把该范围混入本轮 Canvas 对比度调整。

## 第一手资料

### 本地 UE5.2 源码

- `FCanvasTextItemBase` 的颜色、阴影、描边字段与接口：`<ue5_root>/Engine/Source/Runtime/Engine/Public/CanvasItem.h`，约 304-479 行。
- Canvas 阴影和固定四向描边实现、Runtime font cache 测量/绘制路径：`<ue5_root>/Engine/Source/Runtime/Engine/Private/UserInterface/CanvasItem.cpp`，约 830-986 行及后续 Runtime 分支。
- Runtime/Offline 字体缓存定义和 Composite Font：`<ue5_root>/Engine/Source/Runtime/Engine/Classes/Engine/Font.h`，约 15-24、92-98、163-185 行。
- `FSlateFontInfo` 的 Font Material、Outline Settings、Typeface 名称与字号：`<ue5_root>/Engine/Source/Runtime/SlateCore/Public/Fonts/SlateFontInfo.h`。

### Epic 官方文档

- [Text Formatting and Fonts, UE 5.2](https://dev.epicgames.com/documentation/unreal-engine/text-formatting-and-fonts-in-umg-for-unreal-engine?application_version=5.2)
- [Font Asset and Editor](https://dev.epicgames.com/documentation/unreal-engine/font-asset-and-editor-in-unreal-engine?application_version=5.2)
- [Using Fonts with UMG](https://dev.epicgames.com/documentation/unreal-engine/using-fonts-with-umg-in-unreal-engine?application_version=5.2)
- [Font Materials and Outlines](https://dev.epicgames.com/documentation/unreal-engine/font-materials-and-outlines-in-unreal-engine?application_version=5.2)
- [FCanvasTextItem API](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Engine/FCanvasTextItem?application_version=5.2)

### Noto 官方资料

- [Noto CJK 官方仓库](https://github.com/notofonts/noto-cjk)
- [Noto CJK Sans OFL 1.1 许可](https://github.com/notofonts/noto-cjk/blob/main/Sans/LICENSE)
- [Noto CJK Sans 历史与 Regular/Bold TTC](https://github.com/notofonts/noto-cjk/blob/main/Sans/HISTORY.md)
- [Noto 官方使用与许可说明](https://notofonts.github.io/noto-docs/website/use/)


## 实施验证（2026-11-15）

- 插件编译和场景包增量 Build/Stage/Archive 通过；确认 Bold TTC 随包部署。字体来源及 SHA-256 见字体目录 SOURCES.md。
- 实际 1280×720 Vulkan 窗口审核通过：简体标题、中英数字混排、暖金色填充、深色窄描边；繁体长标题在既有单行宽度截断，HUD 按约定绘于标题上方。半透明 HUD 区域内标题会透出，这是保留既有层级与透明度的结果。
- 通过 RPC 验证运行时换字、长标题、清空及恢复；中央提示保持原 Regular 字体。
- 26 项现有 Python 测试通过。证据位于 log/projectairsim/hud-style-review/title.png 和 title-long.png；构建日志位于 log/projectairsim/hud-v1/。
- 用户已认可标题及双标题视觉；没有进行解锁或飞行。

## 左右双标题（2026-11-15）

新增独立右标题，左侧暖金色 #FFD166，右侧亮青色 #6EE0FF。
两侧共享 Bold 字体、36 基准字号、描边、最大宽度、顶部及边缘间距；
右侧文字按实际截断后的宽度靠右对齐，均在 HUD 下层绘制。
SetTitle 的 side 参数支持 left/right，省略默认为 left，空文本仅清空指定侧。
GetDisplayState 新增 title_left/title_right，原 title 保留为左标题别名。
实际运行验证两侧独立更新/清空、旧调用兼容、非法 side 拒绝、
1280×720 右对齐及色彩区分通过；证据为 hud-style-review/titles.png 和 titles-check.json。
