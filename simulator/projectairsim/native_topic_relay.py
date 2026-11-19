#!/usr/bin/env python3
"""Transparent single-peer NNG relay with header-only diagnostics, no capture API.

Connects to upstream only after the real bridge sends a subscription: the manager's
scene-loader must finish using the upstream Pair0 first. Payload is forwarded intact.
"""
import argparse
import json
from pathlib import Path
import signal
import threading
import time
import msgpack
import pynng


def image_header(wire):
    frame=msgpack.unpackb(wire,raw=True)
    if len(frame)<3 or not isinstance(frame[1],bytes):return None
    topic=frame[1].decode()
    if not topic.endswith('/sensors/FrontCamera/scene_camera'):return None
    unpacker=msgpack.Unpacker(raw=False)
    unpacker.feed(frame[2]);fields={}
    for _ in range(unpacker.read_map_header()):
        key=unpacker.unpack()
        if key in ('time_stamp','width','height','encoding'):fields[key]=unpacker.unpack()
        else:unpacker.skip()
    return dict(sample_ns=fields.get('time_stamp'),width=fields.get('width'),height=fields.get('height'),
        payload_bytes=len(frame[2]),wire_bytes=len(wire),topic=topic)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--upstream-port',type=int,default=19889)
    p.add_argument('--listen-port',type=int,default=20189)
    a=p.parse_args();stop=threading.Event();connected=threading.Event();errors=[]
    for sig in(signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.set())
    upstream=pynng.Pair0(recv_timeout=100,send_timeout=1000)
    downstream=pynng.Pair0(listen='tcp://127.0.0.1:'+str(a.listen_port),recv_timeout=100,send_timeout=1000)
    stream=a.output.open('w',buffering=1)
    def pump(source,destination,inspect=False):
        sequence=0
        try:
            while not stop.is_set():
                try:wire=source.recv()
                except (pynng.Timeout,pynng.TryAgain):continue
                received=time.monotonic_ns();wall=time.time_ns()
                if not inspect and not connected.is_set():
                    upstream.dial('tcp://127.0.0.1:'+str(a.upstream_port),block=True)
                    connected.set()
                row=image_header(wire) if inspect else None
                while not stop.is_set():
                    try:destination.send(wire);break
                    except (pynng.Timeout,pynng.TryAgain):continue
                if row is not None:
                    row.update(mono_ns=received,wall_ns=wall,forwarded_mono_ns=time.monotonic_ns(),sequence=sequence)
                    sequence+=1;stream.write(json.dumps(row,separators=(',',':'))+'\n')
        except Exception as error:
            if not stop.is_set():errors.append(str(error));stop.set()
    threads=[threading.Thread(target=pump,args=(downstream,upstream),daemon=True),
             threading.Thread(target=pump,args=(upstream,downstream,True),daemon=True)]
    for thread in threads:thread.start()
    Path(str(a.output)+'.ready').write_text('listening; upstream connection deferred until subscription\n')
    try:
        while not stop.wait(.1):pass
    finally:
        upstream.close();downstream.close()
        for thread in threads:thread.join(3.)
        stream.close()
    if errors:raise RuntimeError('; '.join(errors))


if __name__=='__main__':main()
