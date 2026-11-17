"""AirVLN 场景服务器最小桩（迁移方案偏离 D2）。

上游 AirVLNSimulatorClientTool 在连接 airsim 客户端前，先向 msgpackrpc 场景服务器
（默认 127.0.0.1:30000）调用：
  ping()                          -> 任意成功返回（客户端只测连通）
  reopen_scenes(host, scenes)     -> [True, [ip_bytes, [port, ...]]]
      客户端断言 len(result[1])==2、result[0]!=False、
      ip.decode('utf-8') == str(host)、len(ports)==len(scenes)，
      然后用 (ip, port) 建 airsim.VehicleClient。
  close_scenes(host)              -> 任意成功返回（SimRun 主流程不调用）

本桩把任意 open_scenes 都映射到本机标准 Blocks AirSim 的 RPC 端口，
不做场景切换（Blocks 单场景）。

用法: python scene_server.py [--port 30000] [--airsim-port 45451]
"""
import argparse

from msgpackrpc import Server, Address


class SceneHandler:
    def __init__(self, airsim_port):
        self.airsim_port = airsim_port

    def ping(self):
        return True

    def reopen_scenes(self, host, scenes):
        # host 为客户端传入的 str；ip 必须以 bytes 返回（客户端 .decode）
        n = len(scenes) if scenes else 1
        return [True, [b"127.0.0.1", [self.airsim_port] * n]]

    def close_scenes(self, host):
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=30000)
    ap.add_argument("--airsim-port", type=int, default=45451)
    args = ap.parse_args()

    server = Server(SceneHandler(args.airsim_port))
    server.listen(Address("127.0.0.1", args.port))
    print(f"[scene-stub] listening 127.0.0.1:{args.port}, mapping scenes -> airsim 127.0.0.1:{args.airsim_port}", flush=True)
    server.start()


if __name__ == "__main__":
    main()
