#!/usr/bin/env python3
"""Inspect the running master and sample inputs; never start hardware or publish."""
import argparse
import json
import math
import socket
import time
import rosgraph
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from hdl_localization.msg import ScanMatchingStatus

def inspect(duration):
    result=dict(master_uri=rosgraph.get_master_uri(), observation_only=True,
                hardware_values_confirmed=False, topics={}, tf_authorities={})
    socket.setdefaulttimeout(3)
    master=rosgraph.Master('/scout_dwa_read_only_preflight')
    try:
        publishers, subscribers, services=master.getSystemState()
        types=dict(master.getTopicTypes())
        result['master_reachable']=True
        result['publishers']=dict(publishers)
        result['services']=[name for name,_ in services]
        try: result['use_sim_time']=master.getParam('/use_sim_time')
        except Exception: result['use_sim_time']=False
    except Exception as exc:
        result['master_reachable']=False; result['error']=str(exc)
        return result
    rospy.init_node('scout_dwa_read_only_preflight',anonymous=True,disable_signals=True)
    def receive(name):
        def callback(msg):
            entry=result['topics'].setdefault(name,dict(samples=0,stamps=[],frames=[]))
            entry['samples']+=1
            entry['stamps'].append(msg.header.stamp.to_sec())
            entry['frames'].append(msg.header.frame_id)
            if isinstance(msg,Odometry):
                entry['child_frame_id']=msg.child_frame_id
                entry['last_twist']=[msg.twist.twist.linear.x,msg.twist.twist.angular.z]
            elif isinstance(msg,PointCloud2):
                entry['layout']=dict(width=msg.width,height=msg.height,point_step=msg.point_step,row_step=msg.row_step,
                                     fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in msg.fields])
            elif isinstance(msg,ScanMatchingStatus):
                entry['last_quality']=dict(converged=msg.has_converged,matching_error=msg.matching_error,inlier_fraction=msg.inlier_fraction)
        return callback
    subscriptions=[]
    for name,cls in (('/scout/odom',Odometry),('/scout_odom',Odometry),('/velodyne_points',PointCloud2),
                     ('/localization/odom',Odometry),('/localization/status',ScanMatchingStatus),('/status',ScanMatchingStatus)):
        result['topics'][name]=dict(type=types.get(name),samples=0,stamps=[],frames=[])
        if types.get(name)==cls._type:
            subscriptions.append(rospy.Subscriber(name,cls,receive(name),queue_size=1))
    def transforms(msg):
        caller=msg._connection_header.get('callerid','unknown')
        for transform in msg.transforms:
            edge=transform.header.frame_id+' -> '+transform.child_frame_id
            authorities=result['tf_authorities'].setdefault(edge,[])
            if caller not in authorities: authorities.append(caller)
    for name in ('/tf','/tf_static'):
        subscriptions.append(rospy.Subscriber(name,TFMessage,transforms,queue_size=100))
    end=time.monotonic()+duration
    while time.monotonic()<end and not rospy.is_shutdown(): time.sleep(.05)
    for sub in subscriptions: sub.unregister()
    for entry in result['topics'].values():
        stamps=entry.pop('stamps'); entry['frames']=sorted(set(entry['frames']))
        entry['observed_rate_hz']=max(len(stamps)-1,0)/duration
        entry['zero_stamp_count']=sum(s==0 for s in stamps)
        entry['last_source_age_s']=rospy.Time.now().to_sec()-stamps[-1] if stamps else None
    result['duplicate_tf_authorities']={k:v for k,v in result['tf_authorities'].items() if len(v)>1}
    rospy.signal_shutdown('read-only observation finished')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--duration',type=float,default=5)
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not 0<args.duration<=60: parser.error('duration must be in (0,60]')
    result=inspect(args.duration)
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    raise SystemExit(0 if result['master_reachable'] else 2)
