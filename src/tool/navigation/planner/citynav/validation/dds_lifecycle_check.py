#!/usr/bin/env python3
"""M2 真实 DDS 生命周期测试（迁移方案阶段 5 预留、阶段 7 缺口补齐）。

在 onboard-computer 容器内启动 uav_citynav_planner（无仿真器、无 AS2 栈），
本检查器在工作站镜像容器内经真实 DDS 验证：
  1. 节点发现；
  2. 话题/服务契约（status/events/goal/cancel、llm query/result、
     follow_timed_path action 客户端与 _behavior/modify|stop 服务客户端、
     五目 rgb/depth/camera_info、self_localization pose/twist）；
  3. IDLE 心跳（>=3 条，state=IDLE、ready=false）；
  4. cancel 服务在 IDLE 的幂等应答（"already IDLE"）；
  5. goal 生命周期：goal_accepted + started(RUNNING) -> 无传感器/无 Behavior
     服务器 -> _ready() 为假 -> STOPPED detail=not_ready 收口；
  6. 二次 goal 可再次被接受并同样收口（状态机可重入）。
SIGINT 优雅退出由外层编排脚本验证（docker kill --signal=SIGINT + 退出码）。

结果 JSON 落 --output；任一检查失败即整体 FAIL（exit 1），不放宽门槛。
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

NS = "/drone0"

# 契约方向性事实（graph_probe 实测）：ActionClient 的 send_goal/get_result/
# cancel_goal 是 SERVICE 端点，feedback/status 才是话题；planner 以生产同款
# view_cameras（City* 五目）参数启动。
EXPECTED_TOPICS = {
    "publish": [
        f"{NS}/citynav/status",
        f"{NS}/citynav/events",
        f"{NS}/citynav/llm/query",
    ],
    "subscribe": [
        f"{NS}/citynav/goal",
        f"{NS}/citynav/llm/result",
        f"{NS}/self_localization/pose",
        f"{NS}/self_localization/twist",
        f"{NS}/follow_timed_path/_action/feedback",
        f"{NS}/follow_timed_path/_action/status",
    ] + [
        f"{NS}/sensor_measurements/{cam}/{kind}"
        for cam in ("CityLeft", "CitySlightlyLeft", "CityFront",
                    "CitySlightlyRight", "CityRight")
        for kind in ("rgb/image", "depth_planar/image", "rgb/camera_info")
    ],
}
EXPECTED_SERVICES = [
    f"{NS}/citynav/cancel",
    f"{NS}/follow_timed_path/_action/send_goal",
    f"{NS}/follow_timed_path/_action/get_result",
    f"{NS}/follow_timed_path/_action/cancel_goal",
    f"{NS}/follow_timed_path/_behavior/modify",
    f"{NS}/follow_timed_path/_behavior/stop",
]


class Checker(Node):
    def __init__(self, output: Path):
        super().__init__("citynav_dds_lifecycle_checker")
        self.output = output
        self.statuses = []
        self.events = []
        self.create_subscription(String, f"{NS}/citynav/status",
                                 lambda m: self._tap(self.statuses, m), 10)
        self.create_subscription(String, f"{NS}/citynav/events",
                                 lambda m: self._tap(self.events, m), 10)
        self.goal_pub = self.create_publisher(String, f"{NS}/citynav/goal", 10)
        self.cancel_cli = self.create_client(Trigger, f"{NS}/citynav/cancel")

    @staticmethod
    def _tap(bucket, msg):
        try:
            bucket.append(json.loads(msg.data))
        except ValueError:
            bucket.append({"_bad_json": msg.data[:200]})

    def wait_for(self, predicate, timeout, message):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        raise TimeoutError(message)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    report = {"checks": {}, "started": time.strftime("%Y-%m-%d %H:%M:%S")}

    def check(name, fn):
        try:
            detail = fn()
            report["checks"][name] = {"result": "PASS", "detail": detail}
            print(f"[PASS] {name}: {detail}", flush=True)
        except Exception as exc:  # noqa: BLE001
            report["checks"][name] = {"result": "FAIL", "error": repr(exc)}
            print(f"[FAIL] {name}: {exc!r}", flush=True)

    rclpy.init()
    ck = Checker(args.output)
    try:
        # 1. 节点发现（get_node_names_and_namespaces 返回 (裸名, namespace)）
        def discover():
            ck.wait_for(
                lambda: ("citynav_planner", NS) in ck.get_node_names_and_namespaces(),
                30, "planner node not discovered over DDS")
            return "node /drone0/citynav_planner discovered"
        check("node_discovery", discover)

        # 2. 话题/服务契约。方向性口径：publish 用 per-node 发布者查询；
        #    Humble rclpy 无 per-node 订阅查询，订阅侧取全局话题存在性——
        #    本域内仅 planner 与本检查器两节点，检查器不触碰这些话题，
        #    存在即 planner 所建（goal 话题的方向另由第 5 步的接受行为实证）。
        def contract():
            # 话题/服务的图发现晚于节点名发现（首轮实测），轮询至齐或超时。
            def missing_now():
                pub = {t for t, _ in ck.get_publisher_names_and_types_by_node(
                    "citynav_planner", NS)}
                all_topics = {t for t, _ in ck.get_topic_names_and_types()}
                svc = {s for s, _ in ck.get_service_names_and_types()}
                return ([("publish", t) for t in EXPECTED_TOPICS["publish"] if t not in pub]
                        + [("topic", t) for t in EXPECTED_TOPICS["subscribe"] if t not in all_topics]
                        + [("service", s) for s in EXPECTED_SERVICES if s not in svc])
            ck.wait_for(lambda: not missing_now(), 20,
                        "endpoints never fully discovered")
            residual = missing_now()
            if residual:
                raise AssertionError(f"missing endpoints: {residual}")
            return {"publish_checked": len(EXPECTED_TOPICS["publish"]),
                    "subscribe_checked": len(EXPECTED_TOPICS["subscribe"]),
                    "services": len(EXPECTED_SERVICES)}
        check("topic_service_contract", contract)

        # 3. IDLE 心跳
        def heartbeat():
            ck.wait_for(lambda: len(ck.statuses) >= 3, 6, "fewer than 3 heartbeats")
            snaps = [s for s in ck.statuses if s.get("state") != "ready"]
            if not all(s.get("state") == "IDLE" for s in snaps):
                raise AssertionError(f"non-IDLE heartbeat: {snaps}")
            if not all(s.get("ready") is False for s in snaps):
                raise AssertionError("ready=true without sensors (unexpected)")
            return {"heartbeats": len(snaps), "state": "IDLE", "ready": False}
        check("idle_heartbeat", heartbeat)

        # 4. cancel 幂等（IDLE）
        def cancel_idle():
            if not ck.cancel_cli.wait_for_service(timeout_sec=10):
                raise TimeoutError("citynav/cancel service not available")
            fut = ck.cancel_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(ck, fut, timeout_sec=10)
            res = fut.result()
            if res is None or not res.success or "already IDLE" not in res.message:
                raise AssertionError(f"unexpected cancel response: {res}")
            return res.message
        check("cancel_idempotent_in_idle", cancel_idle)

        # 5. goal -> RUNNING -> not_ready 收口（无传感器路径）
        def goal_cycle(tag):
            ck.events.clear()
            ck.statuses.clear()
            ck.goal_pub.publish(String(data=json.dumps(
                {"instruction": f"dds lifecycle probe {tag}", "use_memory": False})))
            ck.wait_for(
                lambda: any(e.get("event") == "goal_accepted" for e in ck.events),
                10, "goal_accepted event missing")
            # started 是 _on_goal 同步发布的 status（state="started"）；
            # RUNNING 在本无传感器路径下是亚心跳瞬时态（_ready 假立即收口），
            # 0.5 s 心跳不可见属预期，不作为判据。
            ck.wait_for(
                lambda: any(s.get("state") == "started" for s in ck.statuses),
                10, "planner never reported started")
            ck.wait_for(
                lambda: any(s.get("state") == "STOPPED" and s.get("detail") == "not_ready"
                            for s in ck.statuses),
                15, "planner did not converge to STOPPED/not_ready")
            faulted = [s for s in ck.statuses if s.get("state") == "FAULT"]
            if faulted:
                raise AssertionError(f"unexpected FAULT: {faulted}")
            return "goal_accepted -> RUNNING -> STOPPED(not_ready)"
        check("goal_lifecycle_first", lambda: goal_cycle("1"))
        check("goal_lifecycle_reentrant", lambda: goal_cycle("2"))
    finally:
        report["status_events_seen"] = len(ck.statuses)
        report["events_seen"] = len(ck.events)
        passed = all(c["result"] == "PASS" for c in report["checks"].values())
        report["overall"] = "PASS" if passed else "FAIL"
        args.output.joinpath("dds-lifecycle-result.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        print(json.dumps({"overall": report["overall"]}))
        ck.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
