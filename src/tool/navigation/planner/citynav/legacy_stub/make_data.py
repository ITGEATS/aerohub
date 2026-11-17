"""生成 Blocks 冒烟所需的最小数据集（替代 AirVLN Google Drive 数据，范围决策见计划）。

产物（写入 upstream/data/，被上游 .gitignore 忽略）：
  data/gt_by_env/0/val_seen_landmk.json      episodes（指令+landmark+起终点）
  data/mem_graphs/0.pkl                      动作图（空 nx.Graph，explore-only 不使用）
  data/mem_graphs_pruned/0/val_seen/<ep>.pkl 空记忆图（find_closest_node 恒 None）

Blocks 真值坐标来源：deployment/airsim/demos/spf_language_nav/README.md
（OrangeBall (34.25, 33.10, -3.68)，AirSim NED）。
"""
import json
import os
import pickle

import numpy as np
import networkx as nx

UPSTREAM = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "upstream"))

EPISODES = [
    {
        "episode_id": "blocks_ep1",
        "instruction_text": "fly toward the orange ball and stop near it",
        "landmarks": ["orange ball"],
        "start": [-10.0, -10.0, -8.0],
        "goal": [34.25, 33.10, -3.68],
    },
    {
        "episode_id": "blocks_ep2",
        "instruction_text": "go to the orange ball, keep the ball in front and stop beside it",
        "landmarks": ["orange ball"],
        "start": [-20.0, 10.0, -10.0],
        "goal": [34.25, 33.10, -3.68],
    },
    {
        # 多 landmark 链：匹配上游"任一视角可见即推进索引"的入口语义
        # （单 landmark 任务会在首次可见+一步移动后即结束，见 run-20261117-170422 分析）
        "episode_id": "blocks_ep3",
        "instruction_text": "fly past the large red building, then continue toward the orange ball and stop near it",
        "landmarks": ["large red building", "orange ball"],
        "start": [-10.0, -10.0, -8.0],
        "goal": [34.25, 33.10, -3.68],
    },
]


def reference_path(start, goal, step=10.0):
    a = np.array(start)
    b = np.array(goal)
    dist = np.linalg.norm(b - a)
    n = max(2, int(np.ceil(dist / step)))
    pts = [list(a + (b - a) * i / (n - 1)) for i in range(n)]
    return pts


def empty_nav_graph_pickle(path):
    data = {
        "pos": np.zeros((0, 3)),
        "images": [],
        "json_graph": nx.node_link_data(nx.Graph()),
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        pickle.dump(data, f)


def main():
    env = 0
    split = "val_seen"
    episodes = []
    for ep in EPISODES:
        episodes.append({
            "episode_id": ep["episode_id"],
            "instruction": {
                "instruction_text": ep["instruction_text"],
                "landmarks": ep["landmarks"],
            },
            "reference_path": reference_path(ep["start"], ep["goal"]),
            "start_position": ep["start"],
            "start_rotation": [1.0, 0.0, 0.0, 0.0],  # [w,x,y,z]，SimRun 会重排为 [x,y,z,w]
            "goals": [{"position": ep["goal"]}],
        })

    data_root = os.path.join(UPSTREAM, "data")
    gt_dir = os.path.join(data_root, "gt_by_env", str(env))
    os.makedirs(gt_dir, exist_ok=True)
    gt_path = os.path.join(gt_dir, f"{split}_landmk.json")
    with open(gt_path, "w", encoding="utf-8") as f:
        json.dump({"episodes": episodes}, f, indent=2)
    print("wrote", gt_path)

    act_dir = os.path.join(data_root, "mem_graphs")
    os.makedirs(act_dir, exist_ok=True)
    with open(os.path.join(act_dir, f"{env}.pkl"), "wb") as f:
        pickle.dump(nx.Graph(), f)
    print("wrote empty action graph")

    for ep in episodes:
        p = os.path.join(data_root, "mem_graphs_pruned", str(env), split, f"{ep['episode_id']}.pkl")
        empty_nav_graph_pickle(p)
        print("wrote empty memory graph", p)


if __name__ == "__main__":
    main()
