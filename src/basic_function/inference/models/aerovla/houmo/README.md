# AeroVLA 板端 Houmo worker 打包与部署

沿用 YOPO houmo 布局：`build_board_runtime.py` 组装自包含运行目录，上传 RoboPi 后
`start_board.sh` 启动。13 GiB HMM 与 assets 不随包复制——它们已在板端
`/data/local/workspace/aerovla_infer/`（`artifacts.json` 锁定 sha256），运行目录内
以 `aerovla_infer` 符号链接指向该目录。

## 本机准备（Windows/Linux 开发机）

```bash
# 1) 板端 wheelhouse（纯 Python 平台无关 wheel；uvicorn 板端已带，仍随包锁定版本）
python -m pip download --only-binary=:all: --no-deps -d houmo/artifacts/wheels/board \
    fastapi starlette uvicorn

# 2) 组装运行目录（要求 wheelhouse 三件齐备，否则失败）
python houmo/build_board_runtime.py            # 输出 houmo/artifacts/board-runtime
```

## 上板

```bash
hdc file send houmo/artifacts/board-runtime /data/local/aerovla_runtime
hdc shell "ln -sfn /data/local/workspace/aerovla_infer /data/local/aerovla_runtime/aerovla_infer"
hdc shell "sh /data/local/aerovla_runtime/start_board.sh"   # 前台；或 nohup 后台
# 默认 127.0.0.1:8090；跨机访问设 INFERENCE_HOST=0.0.0.0
```

## 验证

```bash
# 模块级 golden（沿用 aerovla_infer/smoke.py，门槛 cosine>=0.98）
hdc shell "cd /data/local/workspace/aerovla_infer && LD_LIBRARY_PATH=/data/local/houmo/lib:... python3 smoke.py"
# worker 契约 + 贪心确定性 + 422 拒绝
hdc shell "cd /data/local/aerovla_runtime && python3 tests/http_smoke.py"
```

## 与旧 service.py 的关系

`/data/local/workspace/aerovla_infer/service.py` 是 2026-10-18 轮的私有 HTTP 服务
（pixel_values 张量入参、worker 内 parse_action）。本 worker 按项目 Inference 契约
重建：图像入参、mosaic 归入模型预处理、动作解析交回 planner。两者可在板端共存
（端口 8080/8090），阶段 3 起 planner 只走 8090；旧服务保留至阶段 5 验收后再清理。
