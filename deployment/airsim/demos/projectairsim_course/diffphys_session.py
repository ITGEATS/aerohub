"""Guard startup after a failed local course simulation."""
import re

PREFIX = 'aerohub-diffphys-course-'
RESET_COMMAND = 'python3 deployment/airsim/demos/projectairsim_course/run_diffphys.py --reset-sim'


def check_start(active, status):
    flights = [n for n in active if 'flight-' in n]
    retained = [n for n in active if n.startswith(PREFIX)]
    if flights:
        raise RuntimeError('任务客户端仍运行：' + ', '.join(flights) + '；等待任务退出，或 Ctrl+C 并等待恢复结束。')
    armed = re.search(r'^\s*arming_state:\s*(\d+)\s*$', status or '', re.M)
    if status is not None and (not armed or armed.group(1) != '1'):
        raise RuntimeError('PX4仍解锁或状态未知；保留执行链。失败会话可显式重置仿真：' + RESET_COMMAND)
    if retained:
        raise RuntimeError('上轮算力容器仍运行：' + ', '.join(retained) + '；清理失败会话：' + RESET_COMMAND)
