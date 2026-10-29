#!/usr/bin/env python3
"""Compare saved input on an isolated ROS master; never publish vehicle topics."""
import argparse,copy,json,os,signal,subprocess,tempfile,threading,time
import rosbag,rospy,tf
from geometry_msgs.msg import PoseStamped,Twist,PolygonStamped,TransformStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--data-dir',required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_MASTER_URI','') != 'http://127.0.0.1:11531':
        parser.error('Use dedicated master http://127.0.0.1:11531, never the vehicle master')
    rospy.init_node('compare_saved_dwa',anonymous=True)
    with rosbag.Bag(os.path.join(args.data_dir,'scout_stationary_snapshot.bag')) as bag:
        msgs={topic:msg for topic,msg,_ in bag.read_messages()}
    previous=json.load(open(os.path.join(args.data_dir,'scout_stationary_analysis.json')))
    goal=msgs['/scout_dwa_analysis/goal']
    publications=[rospy.Publisher('/comparison/'+name,cls,queue_size=1,latch=True)
                  for name,cls in [('map',OccupancyGrid),('footprint',PolygonStamped),
                                   ('odom',Odometry),('goal',PoseStamped)]]
    tfpub=rospy.Publisher('/tf',TFMessage,queue_size=1)
    transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='base_link'
    transform.transform.rotation.w=1
    latest={};lock=threading.Lock();children=[];logs=[]
    def callback(key):
        def receive(msg):
            with lock:latest[key]=msg
        return receive
    subscribers=[]
    config=os.path.abspath(os.path.join(args.data_dir,'dwa_scout_low_speed_test.yaml'))
    result={'input':'saved stationary snapshot, not incident-time replay','cases':[]}
    try:
        for name in ['baseline','improved']:
            subscribers.extend([rospy.Subscriber('/'+name+'/cmd',Twist,callback(name+'_cmd')),
                                rospy.Subscriber('/'+name+'/dwa_planner/candidate_trajectories',MarkerArray,callback(name+'_candidates'))])
            if name=='baseline':
                cmd=['roslaunch','dwa_planner','local_planner.launch','dwa_param:='+config,'robot_param:='+config,
                     'local_goal:=/comparison/goal','use_footprint:=true','use_path_cost:=false','use_scan_as_input:=false']
            else:
                cmd=['roslaunch','scout_dwa','improved_planner.launch','goal_topic:=/comparison/goal']
            cmd+=['ns:='+name,'cmd_vel:=/'+name+'/cmd','global_frame:=base_link',
                  'local_map:=/comparison/map','odom:=/comparison/odom','footprint:=/comparison/footprint']
            log=open(os.path.join(args.data_dir,name+'_offline.log'),'w');logs.append(log)
            children.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
        for name,v in [('stopped',0.),('hypothetical_forward_0.24',.24)]:
            odom=copy.deepcopy(msgs['/scout/odom']);odom.twist.twist.linear.x=v;odom.twist.twist.angular.z=0
            m=copy.deepcopy(msgs['/scout_dwa/local_map']);foot=copy.deepcopy(msgs['/scout_dwa/footprint'])
            with lock:latest.clear()
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                now=rospy.Time.now();transform.header.stamp=now;tfpub.publish(TFMessage([transform]))
                for pub,msg in zip(publications,[m,foot,odom,goal]):
                    msg.header.stamp=now;pub.publish(msg)
                if any(child.poll() is not None for child in children):raise RuntimeError('comparison planner exited')
                time.sleep(.05)
            with lock:data=dict(latest)
            case={'name':name}
            for planner in ['baseline','improved']:
                cmd=data[planner+'_cmd'];markers=data[planner+'_candidates'].markers
                # Planner appends repeated display trajectories; first 63 are actual candidates.
                candidates=markers[:63]
                case[planner]={'command':[cmd.linear.x,cmd.angular.z],
                               'valid_candidates':sum(marker.color.g>.5 for marker in candidates),
                               'candidate_count':len(candidates)}
                if name=='hypothetical_forward_0.24':
                    case[planner]['previously_missed_collision_candidates']=[
                        {'index':i,'allowed':candidates[i].color.g>.5} for i in (21,22)]
            result['cases'].append(case)
        json.dump(result,open(os.path.join(args.data_dir,'improvement_comparison.json'),'w'),indent=2)
        print(json.dumps(result,ensure_ascii=False,indent=2))
    finally:
        for child in children:
            if child.poll() is None:child.send_signal(signal.SIGINT)
        for child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=3)
        for log in logs:log.close()

if __name__=='__main__':main()
