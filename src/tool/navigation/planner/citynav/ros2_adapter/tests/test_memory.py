"""memory 单测：在线建图（新建/20 m 合并/边权重）、find_closest_node、
JSON 落盘往返、dijskra_transform 小图对照、find_optimal_route 小图、
exploit_waypoints（D22 相邻节点逐段）。"""
import numpy as np
import pytest

from uav_citynav_planner.memory import (MemoryGraph, dijskra_transform,
                                        find_closest_node)


def _g(radius=20.0):
    return MemoryGraph(merge_radius=radius)


def test_add_or_merge_new_then_merge():
    g = _g()
    kind, i0 = g.add_or_merge([0.0, 0.0, 1.5], images=(b"F0", b"B0"))
    assert (kind, i0) == ("new", 0)
    kind, i1 = g.add_or_merge([30.0, 0.0, 1.5], images=(b"F1", None))
    assert (kind, i1) == ("new", 1)
    assert g._graph.has_edge(0, 1)
    assert g._graph[0][1]["weight"] == pytest.approx(30.0)
    # 距节点 1 仅 5 m < 20 m → 连边不新建
    kind, hit = g.add_or_merge([35.0, 0.0, 1.5])
    assert (kind, hit) == ("merged", 1)
    assert g.vert_count == 2
    assert g.last_node == 1


def test_merge_chains_previous_node():
    g = _g(radius=10.0)
    g.add_or_merge([0.0, 0.0, 0.0])   # node0
    g.add_or_merge([100.0, 0.0, 0.0])  # node1（距 node0 100 m > 10）
    kind, hit = g.add_or_merge([102.0, 0.0, 0.0])  # 命中 node1
    assert (kind, hit) == ("merged", 1)
    assert g._graph.has_edge(0, 1)
    assert g.vert_count == 2


def test_find_closest_node_threshold():
    g = _g()
    g.add_vertex([0, 0, 0])
    g.add_vertex([10, 0, 0])
    assert find_closest_node(g._graph, [1, 0, 0], thresh=5) == 0
    assert find_closest_node(g._graph, [9, 0, 0], thresh=5) == 1
    node, dist = find_closest_node(g._graph, [50, 0, 0], thresh=5, return_dist=True)
    assert node is None and dist == -1


def test_json_roundtrip(tmp_path):
    g = _g(radius=15.0)
    g.add_or_merge([0.0, 1.0, 2.0], images=(b"front-bytes", b"back-bytes"),
                   text="building")
    g.add_or_merge([40.0, 1.0, 2.0], images=(b"f2", None))
    g.add_edge(0, 1)
    path = str(tmp_path / "mem.json")
    g.save(path)
    g2 = MemoryGraph.load(path)
    assert g2.vert_count == 2
    assert g2.merge_radius == pytest.approx(15.0)
    assert g2._images[0] == [b"front-bytes", b"back-bytes"]
    assert g2._images[1] == [b"f2", None]
    assert g2._texts[0] == "building"
    assert g2._graph.has_edge(0, 1)
    assert np.allclose(g2.node_position(1), [40.0, 1.0, 2.0])
    assert g2.last_node == 1
    with open(path) as f:
        import json
        obj = json.load(f)
    assert "nodes" in obj and "edges" in obj  # JSON 而非 pickle


def test_dijskra_transform_small_graph_manual():
    # 链 0-1-2 权重 1,2；initial=[0,-1e9,-1e9]，alpha=1 →
    # 值 = 0 - 最短路距离：[0,-1,-3]（上游 max-plus 形态）
    g = _g()
    g.add_vertex([0, 0, 0])
    g.add_vertex([1, 0, 0])
    g.add_vertex([3, 0, 0])
    g.add_edge(0, 1)
    g.add_edge(1, 2)
    initial = np.array([0.0, -1e9, -1e9], dtype=np.float32)
    next_vals, prev = dijskra_transform(initial, g, alpha=1.0)
    assert next_vals == pytest.approx([0.0, -1.0, -3.0], abs=1e-5)
    assert prev[1][0] == 0 and prev[2][0] == 1


def test_find_optimal_route_small_graph():
    # 链 0-1-2-3（单位权重），1 个 landmark，node3 强相似 → walk 终点为 3
    g = _g()
    for i in range(4):
        g.add_vertex([i, 0.0, 0.0])
        if i:
            g.add_edge(i - 1, i)
    sim = np.array([[0.0], [0.0], [0.0], [5.0]])
    result = g.find_optimal_route(["tower"], start=0, similarity_matrix=sim,
                                  alpha=1e-4)
    walk_nodes = [a[0] for a in result["walk"]]
    assert walk_nodes[0] == 0 and walk_nodes[-1] == 3


def test_exploit_waypoints_dedup_and_strip_start():
    g = _g()
    for i in range(3):
        g.add_vertex([i * 30.0, 0.0, 1.5])
        if i:
            g.add_edge(i - 1, i)
    walk = [(0, 0), (0, -1), (1, 0), (2, 0), (2, -1)]
    wps = g.exploit_waypoints(walk, [1.0, 0.0, 1.5])
    assert wps == [[30.0, 0.0, 1.5], [60.0, 0.0, 1.5]]


def test_nodes_landmarks_similarity_max_over_images():
    # 假 clip_score：图像行 × 文本列；节点 0 两图取 max（上游 L75 语义）
    g = _g()
    g.add_vertex([0, 0, 0], images=(b"a", b"b"))
    g.add_vertex([10, 0, 0], images=(b"c", None))

    def fake_score(images_b64, texts):
        assert len(images_b64) == 3 and len(texts) == 2
        return [[0.1, 0.9], [0.5, 0.2], [0.7, 0.7]]

    sim = g.nodes_landmarks_similarity(["tower", "park"], fake_score)
    assert sim[0] == pytest.approx([0.5, 0.9])  # max(b,a) per landmark
    assert sim[1] == pytest.approx([0.7, 0.7])
