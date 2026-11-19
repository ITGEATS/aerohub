"""Keep the unchanged startup TCP relay alive independently for landing recovery."""
import argparse
from pathlib import Path
import signal
import sys
import threading
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'deployment/airsim/demos/catora_open_field'))
from sim_startup import StartupRelay
p=argparse.ArgumentParser();p.add_argument('--ready',type=Path,required=True);a=p.parse_args()
stopping=threading.Event()
for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopping.set())
relay=StartupRelay(('127.0.0.1',5190),('127.0.0.1',5180))
a.ready.write_text(str(__import__('os').getpid())+'\n')
try:stopping.wait()
finally:relay.close()
