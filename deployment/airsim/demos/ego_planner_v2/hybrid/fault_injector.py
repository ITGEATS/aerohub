"""Exact owned-process exit or scoped planner gate for one prepared session."""
import json
import os
from pathlib import Path
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool,Trigger
from ament_index_python.packages import get_package_prefix
ROOT=Path(os.environ.get('EGO_SESSION_ROOT',Path(__file__).resolve().parent))
def identity(path):
    fields=(path/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=int(path.name),argv=(path/'cmdline').read_bytes().decode().split('\0'),parent=int(fields[1]),ticks=fields[19],
        domain=[s.decode() for s in (path/'environ').read_bytes().split(b'\0') if s.startswith(b'ROS_DOMAIN_ID=')])
class Injector(Node):
    def __init__(self):
        self.config=json.loads((ROOT/'config.json').read_text());self.vehicle=self.config['vehicle']
        assert self.vehicle==self.config['fault_vehicle']
        super().__init__('acceptance_fault_injector',namespace=self.vehicle)
        self.blocked=dict(odom=False,cloud=False);self.counts={k:dict(received=0,forwarded=0,dropped=0) for k in self.blocked}
        self.events=[];self.pubs={};self.prefix='/acceptance/'+self.vehicle
        for key,kind,suffix in [('odom',Odometry,'odom'),('cloud',PointCloud2,'points')]:
            self.pubs[key]=self.create_publisher(kind,self.prefix+'/'+suffix,qos_profile_sensor_data)
            self.create_subscription(kind,'/'+self.vehicle+'/navigation/'+suffix,lambda m,k=key:self.forward(k,m),qos_profile_sensor_data)
            self.create_service(SetBool,self.prefix+'/block_'+key,lambda q,r,k=key:self.gate(k,q,r))
        self.create_service(Trigger,self.prefix+'/exit_planner',self.kill)
        self.status=self.create_publisher(String,self.prefix+'/injector_status',10)
        self.create_timer(.1,self.report)
    def forward(self,k,msg):
        self.counts[k]['received']+=1
        if self.blocked[k]:self.counts[k]['dropped']+=1
        else:self.pubs[k].publish(msg);self.counts[k]['forwarded']+=1
    def event(self,**kw):
        row=dict(wall=time.time(),run_id=self.config['run_id'],vehicle=self.vehicle,**kw)
        self.events.append(row);self.get_logger().info(json.dumps(row));return json.dumps(row)
    def gate(self,k,q,r):
        if self.config['fault_case']!=k:r.success=False;r.message='Unauthorized fault';return r
        self.blocked[k]=q.data;r.success=True;r.message=self.event(kind=k,blocked=q.data,counts=self.counts[k].copy());return r
    def kill(self,q,r):
        if self.config['fault_case']!='planner_exit':r.success=False;r.message='Unauthorized process exit';return r
        binary=str(Path(get_package_prefix('ego_planner'))/'lib/ego_planner/ego_planner_node')
        allowed=Path(self.config['candidate']) if self.vehicle=='drone0' else Path('/work/install/humble')
        if not Path(binary).resolve().is_relative_to(allowed.resolve()):r.success=False;r.message='Binary outside owned installation';return r
        candidates=[]
        for path in Path('/proc').glob('[0-9]*'):
            try:
                v=identity(path)
                if v['argv'][0]==binary and '__ns:=/'+self.vehicle in v['argv'] and v['parent']==os.getppid() and v['domain']==['ROS_DOMAIN_ID='+str(self.config['domain'])]:candidates.append(v)
            except (OSError,ValueError,IndexError,UnicodeError):pass
        if len(candidates)!=1:r.success=False;r.message='Expected one owned planner: '+repr(candidates);return r
        v=candidates[0]
        if identity(Path('/proc')/str(v['pid']))!=v:r.success=False;r.message='Identity changed';return r
        os.kill(v['pid'],signal.SIGKILL);r.success=True;r.message=self.event(kind='planner_exit',signal='SIGKILL',**v);return r
    def report(self):self.status.publish(String(data=json.dumps(dict(vehicle=self.vehicle,blocked=self.blocked,counts=self.counts,events=self.events))))
if __name__=='__main__':
    rclpy.init();node=Injector()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
