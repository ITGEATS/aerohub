"""Read-only Linux UDP counters for selected processes in the current PID ns."""
import os
from pathlib import Path
import socket
import time


def udp_snapshot(patterns):
    table={}
    for family in ('udp','udp6'):
        for line in Path('/proc/net/'+family).read_text().splitlines()[1:]:
            row=line.split()
            if len(row)<13:continue
            table[row[9]]=dict(family=family,local=row[1],port=int(row[1].split(':')[-1],16),
                rx_queue_bytes=int(row[4].split(':')[1],16),drops=int(row[-1]))
    processes=[]
    for path in Path('/proc').glob('[0-9]*'):
        try:
            args=path.joinpath('cmdline').read_bytes().decode(errors='replace').split('\0')
            if int(path.name)!=os.getpid() and not any(Path(arg).name in patterns for arg in args[:8]):continue
            sockets=[]
            for fd in path.joinpath('fd').iterdir():
                try:link=os.readlink(fd)
                except OSError:continue
                if not link.startswith('socket:['):continue
                inode=link[8:-1]
                if inode not in table:continue
                item=dict(table[inode],inode=inode)
                if int(path.name)==os.getpid():
                    try:
                        duplicate=socket.fromfd(int(fd.name),socket.AF_INET,socket.SOCK_DGRAM)
                        item['receive_buffer_bytes']=duplicate.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)
                        duplicate.close()
                    except OSError:pass
                sockets.append(item)
            processes.append(dict(pid=int(path.name),command=args[:3],sockets=sockets))
        except (OSError,ValueError):continue
    return dict(mono_ns=time.monotonic_ns(),wall_ns=time.time_ns(),processes=processes)
