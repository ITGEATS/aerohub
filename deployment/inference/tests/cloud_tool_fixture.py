"""Actual SPF Inference consumer in a separate ROS client container."""
import json
from pathlib import Path
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from uav_spf_planner.vlm_client import VlmQueryClient

rclpy.init()
node = Node('spf_tool_consumer', namespace='drone0')
executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()
client = VlmQueryClient(node, 'test-vlm')
try:
    if not client._client.wait_for_server(timeout_sec=15.):
        raise RuntimeError('Inference Action not discoverable across containers')
    result = client.request('approach orange ball',3.)
    Path('/cross/client.json').write_text(json.dumps(result,indent=2)+'\n')
    assert result['ok'] and result['point']==[500,500], result
finally:
    executor.shutdown()
    thread.join(timeout=2)
    client._client.destroy()
    node.destroy_node()
    rclpy.shutdown()
