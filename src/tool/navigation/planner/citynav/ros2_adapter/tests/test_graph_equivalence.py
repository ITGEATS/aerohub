"""D24 对照断言：包内 UndirectedGraph vs networkx.Graph。

networkx 仅测试期安装（onboard-computer v0.1.1 容器无该库，运行时零依赖）；
缺失时本文件整体 skip，不影响其余回归。对照面：
  1. 图操作（add_node/add_edge 属性更新、neighbors、get_edge_data 双向共享、
     has_edge、number_of_nodes、nodes(data=True) 与 edges(data=True) 顺序）；
  2. dijskra_transform 在两个同构 MemoryGraph（_graph 分别用 UndirectedGraph
     与 nx.Graph）上的 next/prev 输出逐元素一致（含随机图多 alpha）；
  3. find_optimal_route 的 walk/score 一致。
"""
import numpy as np
import pytest

nx = pytest.importorskip("networkx", reason="对照库（仅测试期；运行时零依赖）")

from uav_citynav_planner.memory import (MemoryGraph, UndirectedGraph,
                                        dijskra_transform, find_closest_node)


def _memory_graph(graph_cls, positions, edges):
    g = MemoryGraph(merge_radius=20.0)
    g._graph = graph_cls()  # 换底层图实现，其余建图逻辑共用
    for p in positions:
        g.add_vertex(p)
    for u, v in edges:
        g.add_edge(u, v)  # 权重=欧氏距离（navigation_graph L45-48 语义）
    return g


def test_graph_ops_match_networkx():
    ours, theirs = UndirectedGraph(), nx.Graph()
    for target in (ours, theirs):
        target.add_node(0, pos=[0.0, 0.0, 0.0])
        target.add_node(1, pos=[1.0, 0.0, 0.0])
        target.add_node(2, pos=[2.0, 0.0, 0.0])  # find_closest_node 要求全节点带 pos（nx 同前提）
        target.add_edge(0, 1, weight=1.0)
        target.add_edge(1, 2, weight=2.0)
        target.add_edge(0, 2, weight=3.0)
    assert ours.number_of_nodes() == theirs.number_of_nodes()
    assert sorted(ours.neighbors(1)) == sorted(theirs.neighbors(1))
    assert ours.get_edge_data(0, 1) == theirs.get_edge_data(0, 1) == {"weight": 1.0}
    assert ours.get_edge_data(5, 1) is None and theirs.get_edge_data(5, 1) is None
    # 双向共享同一 attr dict 对象（nx 同语义：改一侧另一侧可见）
    assert ours[0][1] is ours[1][0]
    assert ours.has_edge(2, 0) == theirs.has_edge(2, 0) is True
    assert (2 in ours) == (2 in theirs)
    assert list(ours.nodes(data=True)) == list(theirs.nodes(data=True))
    assert list(ours.edges(data=True)) == list(theirs.edges(data=True))
    # 重复 add_edge 更新 attrs（nx 语义）
    ours.add_edge(0, 1, weight=9.0)
    theirs.add_edge(0, 1, weight=9.0)
    assert ours.get_edge_data(0, 1) == theirs.get_edge_data(0, 1) == {"weight": 9.0}
    # find_closest_node 消费 nodes(data=True)：两种图返回一致
    assert (find_closest_node(ours, [0.9, 0.0, 0.0], thresh=5)
            == find_closest_node(theirs, [0.9, 0.0, 0.0], thresh=5) == 1)


def test_dijskra_transform_matches_networkx_random_graphs():
    rng = np.random.RandomState(11)
    for trial in range(5):
        n = 12
        positions = rng.uniform(-50.0, 50.0, size=(n, 3))
        edges = [(i, j) for i in range(n) for j in range(i + 1, n)
                 if rng.rand() < 0.3]
        g_ours = _memory_graph(UndirectedGraph, positions, edges)
        g_nx = _memory_graph(nx.Graph, positions, edges)
        initial = rng.uniform(-10.0, 10.0, size=n).astype(np.float32)
        for alpha in (1.0, 1e-4):
            next1, prev1 = dijskra_transform(initial.copy(), g_ours, alpha)
            next2, prev2 = dijskra_transform(initial.copy(), g_nx, alpha)
            assert np.array_equal(next1, next2), f"trial {trial} alpha {alpha}"
            assert prev1 == prev2, f"trial {trial} alpha {prev2}"


def test_find_optimal_route_matches_networkx_backed_graph():
    rng = np.random.RandomState(23)
    n = 10
    positions = rng.uniform(-40.0, 40.0, size=(n, 3))
    # 保证连通：链 + 随机边
    edges = [(i, i + 1) for i in range(n - 1)]
    edges += [(i, j) for i in range(n) for j in range(i + 2, n) if rng.rand() < 0.25]
    g_ours = _memory_graph(UndirectedGraph, positions, edges)
    g_nx = _memory_graph(nx.Graph, positions, edges)
    sim = rng.uniform(0.0, 1.0, size=(n, 2))
    r1 = g_ours.find_optimal_route(["tower", "park"], start=0,
                                   similarity_matrix=sim, alpha=1e-4)
    r2 = g_nx.find_optimal_route(["tower", "park"], start=0,
                                 similarity_matrix=sim, alpha=1e-4)
    assert r1["walk"] == r2["walk"]
    assert np.array_equal(r1["score"], r2["score"])


def test_json_roundtrip_graph_identical(tmp_path):
    # 序列化消费 edges(data=True)：两种底层图落盘结构一致
    positions = [[0.0, 0.0, 1.5], [30.0, 0.0, 1.5], [60.0, 0.0, 1.5]]
    edges = [(0, 1), (1, 2)]
    g1 = _memory_graph(UndirectedGraph, positions, edges)
    g2 = _memory_graph(nx.Graph, positions, edges)
    assert g1.to_json_obj()["edges"] == g2.to_json_obj()["edges"]
    p = str(tmp_path / "g.json")
    g1.save(p)
    g3 = MemoryGraph.load(p)  # 加载后底层为 UndirectedGraph
    assert g3.to_json_obj()["edges"] == g1.to_json_obj()["edges"]
