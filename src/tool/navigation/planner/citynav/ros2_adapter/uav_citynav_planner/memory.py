"""在线拓扑记忆图 + LM-Nav exploit（纯逻辑，无 ROS/torch/cv2/sklearn/networkx
依赖；底层图用包内 UndirectedGraph，D24）。

语义来源：
  - 节点/边/最近节点：upstream/external/lm_nav/navigation_graph.py L21-58
    （add_edge 权重=欧氏距离）与 upstream/utils/maps.py L10-31
    find_closest_node（逐字移植，thresh 由 merge_radius 参数传入）。
  - 落盘：上游 pickle → 本实现 JSON（任务规格：不用 pickle；base64 图像内联）。
  - 20 m 合并（在线建图规则，偏离 D4）：新位置距既有节点 < merge_radius
    则连边不新建；否则新建节点并与上一访问节点连边。
  - dijskra_transform：upstream/external/lm_nav/optimal_route.py L17-37
    逐字移植（函数体不改）。
  - find_optimal_route：optimal_route.py L79-102 移植；CLIP 评分改为注入的
    服务回调（D19：torch CLIP 不进 planner；D23：rectify_and_crop_v2 不移植，
    直接用原图，README 注明）。"A photo of " 前缀逐字保留（L47）。
  - exploit 动作分解：动作图=记忆图本身（偏离 D22，替代作者 pkl 动作图）；
    walk 相邻节点即图边，逐段 calculate_movement_steps 分解。
  - 底层图：上游 navigation_graph.py 用 networkx.Graph；本包改内建
    UndirectedGraph 最小实现（偏离 D24：onboard-computer v0.1.1 容器无
    networkx，约定不重建镜像/不新增基础依赖；dijskra_transform 本就只依赖
    neighbors/get_edge_data 邻接接口，heapq 算法不变；语义等价性由
    tests/test_graph_equivalence.py 对 networkx 对照断言证明，对照仅测试期）。
"""
import base64
import heapq
import json
from typing import List, Optional, Tuple

import numpy as np


class UndirectedGraph:
    """最小无向图（D24 替代 networkx.Graph），只实现本包消费接口，语义对齐：

      add_node(n, **attrs) / add_edge(u, v, **attrs)（重复 add_edge 更新属性，
      双向共享同一 attr dict 对象，同 nx）
      neighbors(n) → 迭代器（插入序）
      get_edge_data(u, v) → attr dict 或 None
      has_edge(u, v) / number_of_nodes() / __contains__ / __getitem__（邻接表）
      nodes(data=True) → (node, attrs) 插入序
      edges(data=True) → (u, v[, attrs])，首次遇到方向输出，算法与
      nx.EdgeView 相同（seen-set）

    欧氏边权由 MemoryGraph.add_edge 计算写入（navigation_graph.py L45-48），
    本类只存 attrs。
    """

    def __init__(self):
        self._adj = {}    # node -> {neighbor: attrs(双向共享)}
        self._attrs = {}  # node -> attrs

    def add_node(self, node, **attrs):
        self._adj.setdefault(node, {})
        self._attrs.setdefault(node, {}).update(attrs)

    def add_edge(self, u, v, **attrs):
        self.add_node(u)
        self.add_node(v)
        data = self._adj[u].get(v)
        if data is None:
            data = {}
            self._adj[u][v] = data
            self._adj[v][u] = data
        data.update(attrs)

    def neighbors(self, node):
        return iter(self._adj[node])

    def get_edge_data(self, u, v):
        return self._adj.get(u, {}).get(v)

    def has_edge(self, u, v):
        return v in self._adj.get(u, {})

    def number_of_nodes(self):
        return len(self._adj)

    def nodes(self, data=False):
        if data:
            return [(n, self._attrs.get(n, {})) for n in self._adj]
        return list(self._adj)

    def edges(self, data=False):
        seen = set()
        out = []
        for u, nbrs in self._adj.items():
            for v, attrs in nbrs.items():
                if v not in seen:
                    out.append((u, v, attrs) if data else (u, v))
            seen.add(u)
        return out

    def __getitem__(self, node):
        return self._adj[node]

    def __contains__(self, node):
        return node in self._adj


def find_closest_node(graph, point, thresh=5, return_dist=False):
    """upstream/utils/maps.py L10-31 逐字移植（graph 为 UndirectedGraph 或
    任何提供 nodes(data=True) 的对象——上游 nx.Graph 同接口，D24）。"""
    min_distance = float('inf')
    closest_node = None
    point = np.array(point)

    for node, data in graph.nodes(data=True):
        n_pos = np.array(data['pos'])
        distance = np.linalg.norm(point - n_pos)
        if distance < min_distance:
            min_distance = distance
            closest_node = node
    if min_distance < thresh:
        if return_dist:
            return closest_node, min_distance
        else:
            return closest_node
    else:
        if return_dist:
            return None, -1
        else:
            return None


def dijskra_transform(initial: np.ndarray, graph, alpha: float
                      ) -> Tuple[np.ndarray, List[Tuple[int, int]]]:
    """upstream/external/lm_nav/optimal_route.py L17-37 逐字移植（不改函数体；
    graph 需提供 .vert_count 与 ._graph.neighbors/get_edge_data["weight"]）。"""
    next = np.copy(initial)
    prev = [(i, -1) for i in range(graph.vert_count)]
    priority_queue = []
    for i in range(len(initial)):
        priority_queue.append((-initial[i], i))
    heapq.heapify(priority_queue)
    while priority_queue:
        (value, node) = heapq.heappop(priority_queue)
        value = -value
        if next[node] != value:
            continue
        for neighbor in graph._graph.neighbors(node):
            weight = alpha * graph._graph.get_edge_data(node, neighbor)["weight"]
            if next[neighbor] < value - weight:
                next[neighbor] = value - weight
                heapq.heappush(priority_queue, (-next[neighbor], neighbor))
                prev[neighbor] = (node, 0)
    return next, prev


class MemoryGraph:
    """在线记忆图。节点 = 位置 + front/back 两图 bytes（可缺）+ 可选文本。

    上游 NavigationGraph 的 _pos/_images/_graph 三结构保留（dijskra_transform
    与 find_optimal_route 按上游属性名消费）；_pos 同步写入节点属性 'pos'
    （作者 pkl 图即如此，find_closest_node 依赖）。底层图为包内
    UndirectedGraph（D24，接口兼容上游 nx.Graph 消费面）。
    """

    def __init__(self, merge_radius: float = 20.0):
        self._pos: List[np.ndarray] = []
        self._images: List[List[Optional[bytes]]] = []
        self._texts: List[Optional[str]] = []
        self._graph = UndirectedGraph()
        self.merge_radius = float(merge_radius)
        self.last_node: Optional[int] = None

    @property
    def vert_count(self):
        return self._graph.number_of_nodes()

    # ---------- 建图 ----------
    def add_vertex(self, pos, images=(None, None), text=None) -> int:
        """navigation_graph.py L50-58 add_vertix（上游拼写保留于注释）。"""
        pos = np.asarray(pos, dtype=np.float64).reshape(1, -1)[0][:3]
        inx = self.vert_count
        self._graph.add_node(inx, pos=pos.tolist())
        self._pos.append(pos)
        self._images.append([images[0], images[1]])
        self._texts.append(text)
        return inx

    def add_edge(self, node, adj_node, weight=None):
        """navigation_graph.py L45-48：权重=欧氏距离。"""
        if weight is None:
            weight = float(np.linalg.norm(self._pos[node] - self._pos[adj_node]))
        self._graph.add_edge(node, adj_node, weight=weight)

    def add_or_merge(self, pos, images=(None, None), text=None):
        """在线建图规则（D4）：距既有节点 < merge_radius → 连边不新建。

        返回 ("merged", idx) 或 ("new", idx)；与上一访问节点（last_node）连边
        形成轨迹拓扑链。
        """
        hit = find_closest_node(self._graph, np.asarray(pos)[:3], thresh=self.merge_radius)
        if hit is not None:
            if self.last_node is not None and self.last_node != hit:
                self.add_edge(self.last_node, hit)
            if text and not self._texts[hit]:
                self._texts[hit] = text
            self.last_node = hit
            return "merged", hit
        inx = self.add_vertex(pos, images=images, text=text)
        if self.last_node is not None:
            self.add_edge(self.last_node, inx)
        self.last_node = inx
        return "new", inx

    def closest_node(self, pos, thresh=None, until=None):
        """最近节点（阈值内），仅考虑 index < until 的节点。

        until 用于阶段 6 修复：exploit 只应命中既往任务记忆（上游 episode
        内图静态），本任务在线 add_or_merge 的自身轨迹节点必须排除，否则
        第二循环即命中刚访问位置 → 自命中 exploit → 多余 goal/提前收口
        （run-20261118d 实测：无既往记忆时旧 min_index 写法完全失效）。
        默认 None=全部节点，保持既有行为。"""
        thresh = self.merge_radius if thresh is None else thresh
        point = np.asarray(pos)[:3]
        best, best_d = None, float("inf")
        end = self.vert_count if until is None else min(int(until), self.vert_count)
        for i in range(0, end):
            d = float(np.linalg.norm(self._pos[i] - point))
            if d < best_d:
                best, best_d = i, d
        return best if best is not None and best_d < thresh else None

    def node_position(self, node) -> np.ndarray:
        return np.asarray(self._pos[int(node)], dtype=np.float64)

    # ---------- 落盘/加载（JSON，不用 pickle） ----------
    def to_json_obj(self) -> dict:
        nodes = []
        for i in range(self.vert_count):
            front, back = self._images[i]
            nodes.append({
                "index": i,
                "pos": [float(v) for v in self._pos[i]],
                "images": {"front": base64.b64encode(front).decode("ascii") if front else None,
                           "back": base64.b64encode(back).decode("ascii") if back else None},
                "text": self._texts[i],
            })
        edges = [[u, v, float(d["weight"])] for u, v, d in self._graph.edges(data=True)]
        return {"interface_version": 1, "merge_radius": self.merge_radius,
                "nodes": nodes, "edges": edges, "last_node": self.last_node}

    def save(self, path: str):
        with open(path, "w") as f:
            json.dump(self.to_json_obj(), f)

    @classmethod
    def load(cls, path: str) -> "MemoryGraph":
        with open(path) as f:
            obj = json.load(f)
        g = cls(merge_radius=obj.get("merge_radius", 20.0))
        for nd in obj["nodes"]:
            imgs = nd.get("images") or {}
            front = base64.b64decode(imgs["front"]) if imgs.get("front") else None
            back = base64.b64decode(imgs["back"]) if imgs.get("back") else None
            g.add_vertex(nd["pos"], images=(front, back), text=nd.get("text"))
        for u, v, w in obj["edges"]:
            g.add_edge(u, v, weight=w)
        g.last_node = obj.get("last_node")
        return g

    # ---------- exploit：CLIP 服务评分 + 图上搜索 ----------
    def nodes_landmarks_similarity(self, landmarks: List[str], clip_score) -> np.ndarray:
        """optimal_route.py L40-76 移植。差异（README D19/D23）：
        - CLIP ViT-L/14 torch 进程内推理 → 注入 clip_score(images_b64, texts)
          服务回调（POST /models/clip/score，similarity[i][j]=图 i 对文本 j）；
        - rectify_and_crop_v2 不移植（D23）：直接用节点原图；
        - "A photo of " 前缀与逐节点 max-over-images 语义逐字保留（L47/L75）。
        """
        result = np.zeros((self.vert_count, len(landmarks)), dtype=np.float64)
        texts = ["A photo of " + desc for desc in landmarks]
        flat_images, owner = [], []
        for i in range(self.vert_count):
            for img in self._images[i]:
                if img:
                    flat_images.append(base64.b64encode(img).decode("ascii"))
                    owner.append(i)
        if not flat_images or not landmarks:
            return result
        similarity = clip_score(flat_images, texts)  # [[n_images] x n_texts]
        sim = np.asarray(similarity, dtype=np.float64)
        for i, row in enumerate(sim):
            node = owner[i]
            result[node, :] = np.maximum(result[node, :], np.asarray(row, dtype=np.float64))
        return result

    def find_optimal_route(self, landmarks: List[str], start: int,
                           similarity_matrix: np.ndarray, alpha: float = 1e-4) -> dict:
        """optimal_route.py L79-102 逐字移植（similarity 由服务回调先算好，
        对应上游 pipeline.full_pipeline 的 landmarks→similarity→搜索链）。"""
        score = np.full(self.vert_count, -1e9, dtype=np.float32)
        score[start] = 0.0
        score, prev1 = dijskra_transform(score, self, alpha)
        prev_tables = [prev1]
        for i in range(len(landmarks)):
            score = score + similarity_matrix[:, i]
            score, prev = dijskra_transform(score, self, alpha)
            prev_tables.append(prev)
        node = int(np.argmax(score))
        table_index = len(prev_tables) - 1
        traversal = [(node, 0)]
        while node != start or table_index > 0:
            node, table_index_change = prev_tables[table_index][node]
            table_index += table_index_change
            traversal.append((node, table_index_change))
        return {"walk": list(reversed(traversal)), "score": score,
                "prev_table": prev_tables, "similarity_matrix": similarity_matrix}

    def exploit_waypoints(self, walk, current_pos) -> List[list]:
        """walk 节点序列 → waypoint 位置序列（D22：动作图=记忆图本身，
        相邻 walk 节点即图边，逐段由 trajectory.calculate_movement_steps
        分解；3 项位姿的 yaw 语义=朝下一目标，最后一段保持当前朝向，
        对应上游 L487 teleport 终点保持 curr orientation）。"""
        nodes = [a[0] for a in walk]  # optimal_route 消费形态 L474
        # walk 回溯含源节点重复项（dijskra prev-table 跨表回退形态）；连续
        # 重复节点归一化去除——上游对重复项分解出零位移段，行为等价、省步数。
        nodes = [n for i, n in enumerate(nodes) if i == 0 or n != nodes[i - 1]]
        positions = [self.node_position(n)[:3].tolist() for n in nodes]
        # 去掉与当前位置重合的起点（walk[0]=命中节点）
        if positions and np.linalg.norm(np.asarray(positions[0]) -
                                        np.asarray(current_pos)[:3]) < self.merge_radius:
            positions = positions[1:]
        return positions
