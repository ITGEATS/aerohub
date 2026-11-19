"""Independent read-only ROS ground observer, including parameter service RTT."""
import json
from pathlib import Path
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rcl_interfaces.srv import GetParameters

root=Path(sys.argv[1]);duration=float(sys.argv[2])
topics={'/clock':'rosgraph_msgs/msg/Clock'}
for ns in ('drone0','drone1'):
    for suffix,kind in [('platform/info','as2_msgs/msg/PlatformInfo'),
        ('fmu/out/vehicle_control_mode','px4_msgs/msg/VehicleControlMode'),
        ('navigation/odom','nav_msgs/msg/Odometry'),('navigation/points','sensor_msgs/msg/PointCloud2'),
        ('ego/native/odom','nav_msgs/msg/Odometry'),('ego/native/points','sensor_msgs/msg/PointCloud2'),
        ('ego/status','std_msgs/msg/String'),('ego_planner/planning/heartbeat','std_msgs/msg/Empty'),
        ('follow_timed_path/_behavior/behavior_status','as2_msgs/msg/BehaviorStatus'),
        ('TakeoffBehavior/_behavior/behavior_status','as2_msgs/msg/BehaviorStatus'),
        ('LandBehavior/_behavior/behavior_status','as2_msgs/msg/BehaviorStatus')]:topics[f'/{ns}/{suffix}']=kind
rclpy.init();node=rclpy.create_node('ego_hybrid_ground_observer')
rows={t:dict(count=0,max_gap=0.,ever_armed=False,states=[]) for t in topics}
events=(root/'observer-events.jsonl').open('w')
def receive(topic,msg):
    row=rows[topic];now=time.monotonic()
    if row['count']:row['max_gap']=max(row['max_gap'],now-row['last'])
    row.update(count=row['count']+1,last=now)
    if hasattr(msg,'flag_armed'):row['ever_armed']|=msg.flag_armed
    if hasattr(msg,'armed'):row['ever_armed']|=msg.armed
    if topic.endswith('/ego/status'):
        state=json.loads(msg.data);row['latest']=state
        if state['state'] not in row['states']:row['states'].append(state['state'])
    if hasattr(msg,'width'):row['points']=msg.width*msg.height
    if hasattr(msg,'header'):row['frame']=msg.header.frame_id
    # Preserve raw small messages; omit multi-megabyte point-cloud payloads.
    if not hasattr(msg,'width'):
        events.write(json.dumps(dict(wall=time.time(),topic=topic,message=message_to_ordereddict(msg)))+'\n')
subs=[node.create_subscription(get_message(kind),topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data) for topic,kind in topics.items()]
deadline=time.monotonic()+duration
while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
graph={}
for topic in [*topics,'/ego_swarm/trajectory',*['/'+v+'/ego/goal' for v in ('drone0','drone1')]]:
    graph[topic]={key:[dict(node=i.node_name,namespace=i.node_namespace,type=i.topic_type,gid=list(i.endpoint_gid)) for i in fn(topic)]
        for key,fn in [('publishers',node.get_publishers_info_by_topic),('subscriptions',node.get_subscriptions_info_by_topic)]}
for row in rows.values():row['receipt_age']=time.monotonic()-row.get('last',0)
services={}
for ns in ('drone0','drone1'):
    path=f'/{ns}/ego_as2_adapter/get_parameters'
    client=node.create_client(GetParameters,path)
    if not client.wait_for_service(timeout_sec=5):services[path]={'error':'unavailable'};continue
    start=time.monotonic();future=client.call_async(GetParameters.Request(names=['vehicle']))
    rclpy.spin_until_future_complete(node,future,timeout_sec=5)
    services[path]=dict(rtt=time.monotonic()-start,response=message_to_ordereddict(future.result()) if future.done() else None)
result=dict(duration=duration,topics=rows,graph=graph,services=services,
    nodes=node.get_node_names_and_namespaces(),all_topics=node.get_topic_names_and_types())
(root/'observer.json').write_text(json.dumps(result,indent=2))
events.close();node.destroy_node();rclpy.shutdown()
print(json.dumps(dict(topics=len(rows),samples=sum(r['count'] for r in rows.values()))))
