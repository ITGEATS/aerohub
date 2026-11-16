"""Independent native-Land recovery in this run's AS2 container."""
import argparse
import json
from pathlib import Path
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from flight import Flight

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
rclpy.init(args=['--ros-args','-r','__ns:=/drone0','-p','use_sim_time:=true'])
node=Flight(a.output);executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
report=dict(disarmed_ground=False)
try:
    node.recover();report.update(node.result)
except Exception as exc:report['error']=repr(exc)
finally:
    node.save();executor.shutdown(timeout_sec=3);thread.join(3);node.destroy_node();rclpy.try_shutdown()
    (a.output/'recovery.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['disarmed_ground'] else 1)
