#!/usr/bin/env python3
"""Render the editable, hand-drawn five-part AeroHub architecture.

Run from any directory with Python 3, PyGObject, librsvg and pycairo:
    python3 docs/images/render_five_part_architecture.py

The PNG is an export of the SVG. No network or image-generation API is used.
"""
from pathlib import Path
from random import Random
from xml.sax.saxutils import escape

WIDTH, HEIGHT = 3840, 2160
HERE = Path(__file__).resolve().parent
STEM = "aerohub_five_part_architecture"
INK = "#20272b"
MUTED = "#56636a"
BLUE = "#437eaf"
ORANGE = "#bc7026"
PURPLE = "#8462ad"
GREEN = "#4c8c71"
rng = Random(20261118)
parts = []


def add(value):
    parts.append(value)


def text(x, y, content, size=30, weight=400, fill=INK, anchor="start",
         hand=False, rotate=None):
    family = "Chilanka, Comic Sans MS, cursive" if hand else "Noto Sans CJK SC, DejaVu Sans, sans-serif"
    transform = f' transform="rotate({rotate} {x} {y})"' if rotate else ""
    add(f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" '
        f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}"{transform}>'
        f'{escape(content)}</text>')


def line(points, color=INK, width=4, dash=None, arrow=True, opacity=1):
    d = "M " + " L ".join(f"{x},{y}" for x, y in points)
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    marker = f' marker-end="url(#arrow-{color[1:]})"' if arrow else ""
    add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" '
        f'stroke-linecap="round" stroke-linejoin="round" opacity="{opacity}"{extra}{marker}/>')


def curve(d, color=INK, width=4, dash=None, arrow=True):
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    marker = f' marker-end="url(#arrow-{color[1:]})"' if arrow else ""
    add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" '
        f'stroke-linecap="round" stroke-linejoin="round"{extra}{marker}/>')


def box(x, y, w, h, fill="white", stroke=INK, radius=18, width=2.6,
        dash=None, sketch=True):
    r = radius
    j = lambda: rng.uniform(-1.5, 1.5) if sketch else 0
    d = (f"M {x+r},{y} Q {x+w/2},{y+j()} {x+w-r},{y} "
         f"Q {x+w},{y} {x+w},{y+r} "
         f"Q {x+w+j()},{y+h/2} {x+w},{y+h-r} "
         f"Q {x+w},{y+h} {x+w-r},{y+h} "
         f"Q {x+w/2},{y+h+j()} {x+r},{y+h} "
         f"Q {x},{y+h} {x},{y+h-r} "
         f"Q {x+j()},{y+h/2} {x},{y+r} Q {x},{y} {x+r},{y} Z")
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    add(f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"{extra}/>')


def panel(x, y, w, h, number, title, subtitle, color, tint):
    box(x, y, w, h, tint, color, 30, 3.8, "15 10")
    box(x+3, y+3, w-6, h-6, f"url(#hatch-{color[1:]})", "none", 28, 0, sketch=False)
    text(x+38, y+70, f"({number}) {title}", 45, 600, hand=True)
    text(x+40, y+114, subtitle, 27, fill=MUTED)


def icon(name, x, y, size=54, color=INK):
    add(f'<use xlink:href="#icon-{name}" x="{x}" y="{y}" width="{size}" height="{size}" '
        f'color="{color}"/>')


def label(x, y, value, color=INK, size=26, width=None):
    if width:
        box(x-width/2, y-size-5, width, size+15, "white", "none", 8, 0, sketch=False)
    text(x, y, value, size, 500, color, "middle")


def chip(x, y, w, value, fill="#f4f5f3", color=INK, size=26):
    box(x, y, w, 48, fill, color, 14, 1.9)
    text(x+w/2, y+33, value, size, 500, color, "middle")


def arrow_down(x, y1, y2, color=INK, dash=None):
    line([(x, y1), (x, y2)], color, dash=dash)


def switch(x, y, left="A", right="B"):
    box(x, y, 192, 58, "#f4f7f4", GREEN, 22, 2)
    box(x+5, y+5, 84, 48, "#d6ebdd", GREEN, 18, 2)
    text(x+47, y+38, left, 27, 600, GREEN, "middle")
    text(x+145, y+38, right, 27, 400, MUTED, "middle")
    icon("switch", x+212, y+7, 45, GREEN)


def make_defs():
    add('<defs>')
    for color in [INK, BLUE, ORANGE, PURPLE, GREEN, MUTED]:
        add(f'<marker id="arrow-{color[1:]}" viewBox="0 0 12 12" refX="10" refY="6" '
            f'markerWidth="10" markerHeight="10" orient="auto-start-reverse" markerUnits="userSpaceOnUse">'
            f'<path d="M 1,1 L 11,6 L 1,11 L 3,6 Z" fill="{color}" stroke="{color}"/></marker>')
    for color in [BLUE, ORANGE, PURPLE, GREEN, "#be7e69"]:
        add(f'<pattern id="hatch-{color[1:]}" width="22" height="22" patternUnits="userSpaceOnUse">'
            f'<path d="M -6,6 L 6,-6 M 0,22 L 22,0 M 16,28 L 28,16" '
            f'stroke="{color}" stroke-opacity=".075" stroke-width="1.1"/></pattern>')
    drawings = {
        "search": '<circle cx="21" cy="21" r="13"/><path d="M31 31l12 12M11 21h20M21 8c-8 8-8 18 0 26M21 8c8 8 8 18 0 26"/>',
        "robot": '<rect x="7" y="14" width="34" height="27" rx="8"/><path d="M24 14V6M20 6h8M2 22v12M46 22v12M17 34h14"/><circle cx="16" cy="24" r="2"/><circle cx="32" cy="24" r="2"/>',
        "store": '<path d="M6 20v24h36V20M3 18l5-13h32l5 13M3 18q5 9 10 0 5 9 11 0 6 9 11 0 5 9 10 0M20 44V29h12v15"/>',
        "human": '<circle cx="20" cy="11" r="7"/><path d="M6 39v-8q0-10 14-10t14 10v8M31 27l13 9-8 2-2 8z"/>',
        "book": '<path d="M24 12Q11 5 3 10v31q10-5 21 2 11-7 21-2V10q-8-5-21 2v31M8 17l11 1M8 24l11 1M8 31l11 1M29 18l10-1M29 25l10-1M29 32l10-1"/>',
        "card": '<path d="M9 3h21l9 9v33H9zM29 3v10h10M15 21h17M15 28h17M15 35h11"/>',
        "pc": '<rect x="3" y="5" width="42" height="29" rx="3"/><path d="M18 34v8M30 34v8M11 43h26M8 27h32"/>',
        "board": '<rect x="7" y="7" width="34" height="34" rx="4"/><rect x="17" y="17" width="14" height="14" rx="2"/><path d="M15 2v5M24 2v5M33 2v5M15 41v5M24 41v5M33 41v5M2 15h5M2 24h5M2 33h5M41 15h5M41 24h5M41 33h5"/>',
        "route": '<path d="M7 39q29 3 13-13T39 9" stroke-dasharray="4 4"/><circle cx="7" cy="39" r="4"/><path d="M31 8h10v12M41 8l-9 9"/>',
        "target": '<path d="M4 15V4h11M33 4h11v11M44 33v11H33M15 44H4V33"/><circle cx="24" cy="24" r="9"/><path d="M24 11v6M24 31v6M11 24h6M31 24h6"/>',
        "compass": '<circle cx="24" cy="24" r="20"/><path d="M33 13l-5 15-14 7 7-16zM21 19l7 9"/>',
        "tracking": '<path d="M3 16h7M3 24h10M3 32h7"/><circle cx="29" cy="24" r="14"/><path d="M29 7v8M29 33v8M12 24h8M38 24h8"/><circle cx="29" cy="24" r="4"/>',
        "map": '<path d="M3 10l14-6 14 6 14-6v34l-14 6-14-6-14 6zM17 4v34M31 10v34"/><path d="M9 25q8-15 16 0t14-1" stroke-dasharray="3 4"/>',
        "inference": '<circle cx="24" cy="9" r="4"/><circle cx="10" cy="24" r="4"/><circle cx="38" cy="24" r="4"/><circle cx="24" cy="39" r="4"/><path d="M21 13l-8 8M27 13l8 8M13 28l8 8M35 28l-8 8M14 24h20M24 13v22"/>',
        "control": '<circle cx="24" cy="24" r="17"/><path d="M8 11l7 7M33 30l7 7M7 28l11-3M31 18l10-5"/><circle cx="24" cy="24" r="6"/>',
        "drone": '<path d="M13 13l22 22M13 35l22-22"/><rect x="18" y="18" width="12" height="12" rx="4"/><ellipse cx="10" cy="10" rx="9" ry="6"/><ellipse cx="38" cy="10" rx="9" ry="6"/><ellipse cx="10" cy="38" rx="9" ry="6"/><ellipse cx="38" cy="38" rx="9" ry="6"/>',
        "trophy": '<path d="M13 5h22v14q0 13-11 13T13 19zM13 9H4v9q0 8 11 8M35 9h9v9q0 8-11 8M24 32v9M14 44h20"/>',
        "play": '<rect x="3" y="7" width="42" height="34" rx="5"/><path d="M19 15l15 9-15 9z"/>',
        "shield": '<path d="M24 3l17 7v14q-2 14-17 22Q9 38 7 24V10zM15 24l7 7 13-16"/>',
        "sensor": '<rect x="7" y="16" width="34" height="26" rx="5"/><circle cx="24" cy="28" r="8"/><path d="M14 16l4-7h12l4 7M3 8q4-6 10-5M35 3q6-1 10 5"/>',
        "switch": '<path d="M4 15h36l-8-8M44 33H8l8 8"/><path d="M40 15l-8 8M8 33l8-8"/>',
        "chart": '<path d="M5 5v38h39M13 35V23h6v12M25 35V14h6v21M37 35V5h6v30"/>',
        "download": '<path d="M24 4v28M14 22l10 10 10-10M7 32v12h34V32"/>',
        "papers": '<path d="M13 4h27v34H13zM7 11H3v34h29v-4M19 12h15M19 20h15M19 28h10"/>',
    }
    for name, drawing in drawings.items():
        add(f'<symbol id="icon-{name}" viewBox="0 0 48 48">'
            f'<g fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" '
            f'stroke-linejoin="round">{drawing}</g></symbol>')
    add('</defs>')


def offline():
    panel(40, 190, 1100, 1780, 1, "Offline Adaptation",
          "发现工作 → 技能驱动适配 → PC / 开发板交付", BLUE, "#f3f8fc")
    box(100, 380, 975, 300, "#ffffff", BLUE)
    icon("search", 140, 415, 64)
    text(225, 462, "Auto Search Agent", 43, 600, hand=True)
    icon("papers", 965, 412, 66)
    text(145, 526, "持续抓取无人机相关工作", 34, 600)
    text(145, 580, "综合效果、会议 / 期刊与已适配基线", 30)
    chip(145, 610, 245, "Discover", "#ecf3f9", BLUE)
    chip(417, 610, 245, "Compare", "#ecf3f9", BLUE)
    chip(690, 610, 245, "Select", "#ecf3f9", BLUE)

    arrow_down(315, 680, 770)
    label(315, 733, "持续上架", BLUE, 27, 155)
    arrow_down(865, 680, 790)
    label(865, 741, "自动筛选", BLUE, 27, 155)

    box(100, 770, 465, 360, "#ffffff", BLUE)
    icon("store", 133, 801, 54)
    text(205, 840, "Work Store", 38, 600, hand=True)
    text(135, 889, "前端 · 浏览无人机工作", 28)
    for x in [137, 270, 403]:
        box(x, 916, 115, 70, "#f2f6fa", "#8a9ead", 10, 1.5)
        icon("papers", x+36, 928, 44)
    box(135, 1015, 265, 76, "#dcebf7", BLUE, 18, 2.5)
    icon("download", 153, 1030, 44)
    text(291, 1066, "Download", 33, 600, BLUE, "middle", hand=True)
    icon("human", 434, 1030, 62)

    box(650, 790, 425, 245, "#ffffff", BLUE)
    text(862, 851, "Better candidate?", 33, 600, INK, "middle", hand=True)
    icon("chart", 690, 893, 57)
    text(785, 921, "优于已适配工作", 29, 600)
    text(862, 985, "效果 + 发表质量", 27, 400, MUTED, "middle")

    line([(330, 1130), (330, 1265)])
    label(330, 1191, "人类触发", INK, 30, 180)
    text(330, 1228, "点击下载", 26, fill=MUTED, anchor="middle")
    line([(862, 1035), (862, 1265)])
    label(862, 1159, "自动触发", INK, 30, 180)
    text(862, 1199, "发现更优工作时", 26, fill=MUTED, anchor="middle")

    box(100, 1265, 975, 250, "#fffaf0", ORANGE, 22, 3)
    icon("robot", 145, 1300, 75)
    text(245, 1352, "Offline Agent", 47, 600, hand=True)
    text(145, 1416, "按适配 Skills 检查、迁移、构建与验证", 32)
    text(145, 1473, "适配 PC / 开发板 · 生成功能卡片", 31)

    box(100, 1610, 430, 270, "#f8f3fc", PURPLE)
    icon("book", 130, 1640, 60)
    text(210, 1683, "Adaptation Skills", 34, 600, hand=True)
    text(135, 1746, "适配中总结的可复用方法", 28)
    text(135, 1797, "环境 · 接口 · 构建 · 验证", 27)
    text(135, 1843, "随适配经验持续迭代", 27, fill=PURPLE)
    arrow_down(235, 1610, 1515, PURPLE)
    label(210, 1570, "指导适配", PURPLE, 25, 136)
    arrow_down(435, 1515, 1610, PURPLE, "8 7")
    label(449, 1570, "经验沉淀", PURPLE, 25, 136)

    arrow_down(862, 1515, 1610)
    label(862, 1570, "交付产物", INK, 27, 170)
    box(650, 1610, 425, 270, "#f4faf6", GREEN)
    icon("pc", 685, 1644, 65)
    icon("board", 791, 1644, 65)
    icon("card", 958, 1644, 65)
    text(862, 1760, "PC 或 开发板", 32, 600, INK, "middle")
    text(862, 1811, "适配包 + 功能卡片", 30, 500, INK, "middle")
    text(862, 1854, "Tool / 基础能力接入", 26, 400, MUTED, "middle")


def runtime():
    panel(1260, 190, 1460, 1780, 2, "Online Runtime",
          "人类任务 → 工具调用 → 基础执行与平台", ORANGE, "#fffaf0")
    mainx, mainw = 1410, 1240
    box(mainx, 350, mainw, 110)
    icon("drone", 1443, 373, 60)
    text(1530, 399, "Application", 37, 600, hand=True)
    text(1843, 399, "巡检 · 建图 · 自定义应用", 30)
    text(1530, 438, "Human mission / 人类目标", 27, fill=MUTED)
    arrow_down(2030, 460, 505)
    box(mainx, 505, mainw, 155, "#fffdf7")
    icon("robot", 1443, 538, 61)
    text(1530, 558, "Mission / Online Agent", 39, 600, hand=True)
    text(1530, 607, "拆解任务 · 依据功能卡片选工具 · 消费反馈", 30)
    text(1530, 645, "任务策略 / 权限 · 人工接管", 26, fill=MUTED)
    arrow_down(2030, 660, 725)

    box(mainx, 725, mainw, 255, "#f2faf5", GREEN, 20, 2.7)
    text(1443, 770, "Tools", 36, 600, hand=True)
    icon("card", 2120, 738, 36, GREEN)
    text(2170, 767, "功能卡片 / Capability card", 26, fill=GREEN)
    tools = [
        (1440, "route", "Navigation", "导航"),
        (1738, "target", "Object Detection", "目标检测"),
        (2036, "compass", "Exploration", "探索"),
        (2334, "tracking", "Tracking", "跟踪"),
    ]
    for x, pictogram, title, cn in tools:
        box(x, 795, 281, 123, "white", GREEN, 16, 2)
        icon(pictogram, x+16, 808, 42)
        icon("card", x+236, 807, 29, GREEN)
        text(x+140, 868, title, 28 if len(title)>12 else 31, 500, INK, "middle", hand=True)
        text(x+140, 905, cn, 26, 500, MUTED, "middle")
    text(1444, 956, "运行时约束：资源仲裁 · 超时 / 取消 · 执行交接", 28, fill=GREEN)

    arrow_down(2030, 980, 1045)
    label(2030, 1023, "运动执行请求", INK, 26, 205)
    box(mainx, 1045, mainw, 140)
    text(1444, 1093, "Basic Behaviors", 36, 600, hand=True)
    text(1815, 1093, "基础执行", 29, fill=MUTED)
    text(1444, 1147, "Takeoff · Land · GoTo · FollowPath (+ FollowTimedPath)", 29)
    text(2444, 1093, "FollowReference", 24, fill=MUTED, anchor="middle")

    box(mainx, 1255, mainw, 445, "#ffffff", "#aeb4a8", 20, 2)
    text(1444, 1302, "Basic Functions", 35, 600, hand=True)
    text(1830, 1302, "基础能力 · 兼容接口下选择 / 切换实现", 27, fill=MUTED)
    function_cards = [
        (1440, 315, "State Estimator", "自身状态"),
        (1775, 315, "Perception", "感知与建图"),
    ]
    for x, w, title, cn in function_cards:
        box(x, 1330, w, 272, "#eff8f3", GREEN, 16, 2.3)
        text(x+w/2, 1376, title, 32, 600, INK, "middle", hand=True)
        text(x+w/2, 1415, cn, 26, 500, MUTED, "middle")
        switch(x+28, 1441)
        text(x+w/2, 1535, "选择 / 切换", 26, 500, GREEN, "middle")
    text(1597, 1578, "Pose · Twist · TF", 25, 500, INK, "middle")
    text(1932, 1578, "Maps · Detections", 25, 500, INK, "middle")

    box(2110, 1330, 220, 272, "#f8f5fc", PURPLE, 16, 2.2)
    icon("inference", 2193, 1350, 55)
    text(2220, 1450, "Inference", 31, 600, INK, "middle", hand=True)
    text(2220, 1493, "模型推理", 26, 500, MUTED, "middle")
    text(2220, 1550, "Model services", 24, 400, MUTED, "middle")

    box(2350, 1330, 270, 272, "#fff8e8", ORANGE, 16, 2.3)
    icon("control", 2457, 1350, 55)
    text(2485, 1444, "Motion", 31, 600, INK, "middle", hand=True)
    text(2485, 1485, "Controller", 31, 600, INK, "middle", hand=True)
    text(2485, 1536, "运动参考 + 状态", 25, 500, MUTED, "middle")
    text(2485, 1578, "控制指令", 26, 500, INK, "middle")
    line([(2525, 1185), (2525, 1230), (2485, 1230), (2485, 1330)])
    label(2515, 1223, "运动参考", INK, 23, 130)


    box(mainx, 1750, 870, 132, "#f1f6fb", BLUE, 18, 2.3)
    icon("sensor", 1440, 1785, 58)
    text(1530, 1798, "Sensors / Data Bridges", 33, 600, hand=True)
    text(1530, 1845, "IMU · Camera · LiDAR · 时间与坐标", 27, fill=MUTED)
    line([(1600, 1750), (1600, 1720), (1718, 1720), (1718, 1602)], BLUE, 3, "8 7")
    line([(1905, 1750), (1905, 1720), (2054, 1720), (2054, 1602)], BLUE, 3, "8 7")

    box(2350, 1750, 270, 132, "#fff8e8", ORANGE, 18, 2.3)
    text(2485, 1796, "Platform / PX4", 29, 600, INK, "middle", hand=True)
    text(2485, 1840, "平台与飞控", 27, 500, MUTED, "middle")
    arrow_down(2485, 1602, 1750)
    label(2485, 1720, "控制", INK, 25, 82)
    text(1440, 1930, "AirSim + PX4 SITL / UAV / Other AS2 platforms", 25)
    box(2305, 1897, 345, 60, "#fff8f4", "#be7e69", 12, 1.6)
    icon("shield", 2316, 1907, 38, "#9a6854")
    text(2367, 1923, "Emergency Handling", 23, 500, hand=True)
    text(2367, 1948, "独立保护 / 人工接管", 23, fill=MUTED)
    line([(2350, 1816), (2280, 1816)], BLUE, 3, "8 7")

    # Logical data interfaces, separate from the motion execution path.
    box(1288, 800, 80, 1082, "#fff2df", ORANGE, 17, 1.7)
    text(1335, 1510, "共享数据 / State · Maps · Sensors · Inference", 28,
         500, ORANGE, "middle", rotate=-90)
    line([(1368, 854), (1440, 854)], ORANGE, 3.7)
    line([(1368, 1110), (1410, 1110)], ORANGE, 3.7)
    line([(1597, 1602), (1597, 1629), (1387, 1629), (1387, 1564), (1368, 1564)], ORANGE, 3)
    line([(1932, 1602), (1932, 1640), (1737, 1640), (1737, 1629), (1597, 1629)], ORANGE, 3, arrow=False)
    line([(2220, 1602), (2220, 1640), (1932, 1640)], ORANGE, 3, arrow=False)
    line([(2220, 1640), (2374, 1640), (2374, 1602)], ORANGE, 3)
    text(2330, 1677, "State", 24, fill=ORANGE, anchor="middle")
    line([(1410, 1816), (1368, 1816)], ORANGE, 3)
    # Feedback is a distinct interface return path, not another scheduler.
    line([(2650, 1110), (2693, 1110), (2693, 583), (2650, 583)], BLUE, 3, "9 7")
    text(2681, 862, "Feedback / Result", 23, 500, BLUE, "middle", rotate=-90)
    text(2653, 1460, "ROS 2 · Action / Service / Topic / TF", 22,
         400, MUTED, "middle", rotate=-90)


def benchmark():
    panel(2820, 190, 980, 630, 3, "Benchmark",
          "已适配、同类型工作的公平比较", GREEN, "#f3faf5")
    box(2870, 345, 880, 185, "#ffffff", GREEN)
    icon("chart", 2908, 381, 66)
    text(3002, 401, "Same-type Benchmark", 39, 600, hand=True)
    text(3002, 454, "同任务 / 同平台 · 质量与效率评测", 30)
    text(3002, 502, "成功率 · 误差 · 时延 · 资源", 28, fill=MUTED)
    arrow_down(3310, 530, 585)
    box(2870, 585, 880, 175, "#ffffff", GREEN)
    icon("trophy", 2910, 622, 67)
    text(3002, 641, "Leaderboard UI", 40, 600, hand=True)
    text(3002, 695, "前端打榜 · 同类排名与指标对比", 30)
    text(3002, 736, "按工作类型与评测条件分榜", 26, fill=MUTED)


def training():
    panel(2820, 880, 980, 300, 4, "Training",
          "未来方向 · 目前仍在构思阶段", PURPLE, "#f8f4fc")
    icon("inference", 2885, 1030, 77, PURPLE)
    text(3010, 1067, "Concept stage / 构思中", 37, 600, PURPLE)
    text(3010, 1120, "目标、数据与训练流程待定义", 29, fill=MUTED)


def gallery():
    panel(2820, 1240, 980, 730, 5, "Gallery",
          "自动展示已适配工作在适配平台上的效果", "#be7e69", "#fff7f2")
    box(2870, 1400, 880, 505, "#ffffff", "#be7e69", 18, 2.5)
    line([(2870, 1461), (3750, 1461)], "#be7e69", 1.7, arrow=False)
    for x in [2895, 2917, 2939]:
        add(f'<circle cx="{x}" cy="1430" r="5" fill="#be7e69"/>')
    text(2980, 1441, "Adapted Works Gallery", 32, 600, hand=True)
    text(3706, 1440, "界面示意", 23, fill=MUTED, anchor="end")
    chip(2910, 1483, 140, "PC", "#fff3e9", "#9a6854", 24)
    chip(3070, 1483, 155, "开发板", "#fff3e9", "#9a6854", 24)
    text(3705, 1517, "Work / Platform", 25, fill=MUTED, anchor="end")
    tile_data = [(2910, "轨迹 / 飞行"), (3180, "地图 / 定位"), (3450, "检测 / 跟踪")]
    for x, title in tile_data:
        box(x, 1570, 245, 196, "#faf8f4", "#ab8c7c", 14, 1.6)
        icon("play", x+187, 1719, 33, "#9a6854")
        text(x+122, 1812, title, 27, 500, INK, "middle")
    curve("M 2935,1712 Q 2985,1585 3020,1686 T 3125,1610", BLUE, 3.2, "7 7", arrow=False)
    icon("drone", 2985, 1613, 60)
    for i in range(40):
        x = 3205 + rng.uniform(0, 193)
        y = 1604 + rng.uniform(0, 113)
        add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{rng.uniform(2,4):.1f}" fill="{GREEN}" opacity=".65"/>')
    icon("map", 3256, 1615, 62)
    icon("human", 3535, 1620, 59)
    box(3509, 1595, 117, 115, "none", GREEN, 3, 2, sketch=False)
    line([(3490, 1725), (3535, 1718), (3568, 1705)], BLUE, 2.5, "5 5", arrow=False)
    text(3310, 1864, "自动归档：视频 · 轨迹 · 地图 · 运行指标", 28, 500, MUTED, "middle")


def global_flows():
    # Delivered packages and cards enter the runtime integration boundary.
    line([(1075, 1743), (1200, 1743), (1200, 745), (1410, 745)], INK, 4.5)
    text(1183, 1244, "部署适配包 + 功能卡片", 28, 500, INK, "middle", rotate=-90)

    # Same-type evaluation and automatic showcase consume deployed runtime work.
    line([(2720, 447), (2870, 447)], INK, 4.5)
    label(2790, 419, "评测", INK, 25, 65)
    line([(2720, 1625), (2870, 1625)], INK, 4.5)
    label(2790, 1589, "运行效果", INK, 24, 110)

    # Evaluation informs future selection. The top return path avoids panel interiors.
    line([(3750, 681), (3820, 681), (3820, 151), (1173, 151),
          (1173, 541), (1075, 541)], GREEN, 3.5, "12 9")
    label(2400, 146, "Benchmark feedback · 评测结果反馈自动筛选", GREEN, 28, 765)

    # One consistent legend and explicit target-state qualification.
    text(70, 2040, "图例", 27, 600)
    line([(165, 2028), (235, 2028)], INK, 4)
    text(255, 2038, "调用 / 交付", 26)
    line([(520, 2028), (590, 2028)], ORANGE, 3.5)
    text(610, 2038, "数据访问", 26, fill=ORANGE)
    line([(865, 2028), (935, 2028)], BLUE, 3.5, "9 7")
    text(955, 2038, "状态 / 执行反馈", 26, fill=BLUE)
    line([(1300, 2028), (1370, 2028)], PURPLE, 3.5, "9 7")
    text(1390, 2038, "经验更新", 26, fill=PURPLE)
    icon("card", 1732, 2007, 37, GREEN)
    text(1790, 2038, "功能卡片", 26, fill=GREEN)
    text(2230, 2038, "A / B 选择器：兼容接口下切换实现", 27, fill=GREEN)
    text(2230, 2083, "State：单一权威源；Perception：匹配表示 / 显式转换", 25, fill=GREEN)
    text(70, 2110, "目标设计；具体实现与验收状态以组件文档为准。Training 尚在构思阶段。", 27, fill=MUTED)
    text(3765, 2110, "AeroHub · Architecture v2", 26, fill=MUTED, anchor="end")


def main():
    add('<?xml version="1.0" encoding="UTF-8"?>')
    add(f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
        f'width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}" role="img" '
        f'aria-labelledby="figure-title figure-desc">')
    add('<title id="figure-title">AeroHub: Offline Adaptation, Online Runtime, Benchmark, Training and Gallery</title>')
    add('<desc id="figure-desc">Target architecture. Human download and automatic selection both trigger '
        'the Offline Agent, guided by adaptation skills, to deliver PC or board packages and capability cards. '
        'Online Runtime includes four tools, basic behaviors, interchangeable state-estimation and perception '
        'providers, inference, control, sensors and platform interfaces. Same-type benchmarks publish a leaderboard '
        'and inform selection. Gallery automatically presents runtime effects. Training remains conceptual.</desc>')
    make_defs()
    add('<rect width="3840" height="2160" fill="white"/>')
    text(60, 79, "AeroHub / UAV Ecosystem", 65, 600, hand=True)
    text(62, 120, "五部分目标架构 · Discover, adapt, run, evaluate and showcase", 29, fill=MUTED)
    box(3235, 38, 565, 70, "#fafaf7", "#89938c", 18, 2, "9 6")
    text(3517, 83, "TARGET ARCHITECTURE", 29, 600, MUTED, "middle", hand=True)
    offline()
    runtime()
    benchmark()
    training()
    gallery()
    global_flows()
    add('</svg>')
    svg_path = HERE / f"{STEM}.svg"
    png_path = HERE / f"{STEM}.png"
    svg_path.write_text("\n".join(parts) + "\n", encoding="utf-8")

    import gi
    gi.require_version("Rsvg", "2.0")
    from gi.repository import Rsvg
    import cairo
    handle = Rsvg.Handle.new_from_file(str(svg_path))
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, WIDTH, HEIGHT)
    context = cairo.Context(surface)
    viewport = Rsvg.Rectangle()
    viewport.x, viewport.y, viewport.width, viewport.height = 0, 0, WIDTH, HEIGHT
    handle.render_document(context, viewport)
    surface.write_to_png(str(png_path))
    print(f"Rendered {svg_path.name} and {png_path.name}: {WIDTH}x{HEIGHT}")


if __name__ == "__main__":
    main()
