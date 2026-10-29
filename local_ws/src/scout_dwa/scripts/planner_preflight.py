#!/usr/bin/env python3
"""Start an unmodified planner only after validating its effective parameters."""
import subprocess
import sys
import os
import threading
import yaml
import rospy
from scout_dwa.msg import IntegrationHealth
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_integration_config import validate
from std_srvs.srv import SetBool, Trigger, TriggerResponse

def main():
    rospy.init_node('planner_preflight')
    effective = rospy.get_param('~')
    with open(effective.pop('adapter_config_file'), encoding='utf-8') as stream:
        adapter = yaml.safe_load(stream)
    adapter['target_frame'] = effective['ROBOT_FRAME']
    errors = validate(effective, adapter)
    if errors:
        for error in errors:
            rospy.logfatal('configuration error: %s', error)
        return 2
    planner_name = rospy.get_namespace() + 'dwa_planner'
    rospy.set_param(planner_name, effective)
    remaps = [a for a in sys.argv[1:] if ':=' in a and not a.startswith('__')]
    command = ['rosrun', 'dwa_planner', 'dwa_planner',
               '__name:=dwa_planner', '__ns:='+rospy.get_namespace()] + remaps
    child = subprocess.Popen(command)
    lock = threading.RLock()
    def terminate():
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait()
    def restart(_):
        nonlocal child
        with lock:
            try:
                rospy.wait_for_service('/cmd_vel_guard/set_enabled', timeout=2)
                rospy.ServiceProxy('/cmd_vel_guard/set_enabled', SetBool)(False)
                rospy.wait_for_service('/input_gateway/commit_new_session', timeout=2)
                terminate()
                result = rospy.ServiceProxy('/input_gateway/commit_new_session', Trigger)()
                if not result.success:
                    child = subprocess.Popen(command)
                    return TriggerResponse(success=False, message=result.message)
                child = subprocess.Popen(command)
                return TriggerResponse(success=True, message='new planner session started; guard stays disabled')
            except (rospy.ROSException, rospy.ServiceException) as exc:
                return TriggerResponse(success=False, message=str(exc))
    rospy.Service('~restart_session', Trigger, restart)
    pub = rospy.Publisher('/scout_dwa/config_health', IntegrationHealth, queue_size=1)
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            with lock:
                if child.poll() is not None:
                    break
                h = IntegrationHealth()
                h.header.stamp = rospy.Time.now(); h.valid = True
                h.reason = 'effective planner configuration validated'
                pub.publish(h)
            rate.sleep()
    finally:
        h = IntegrationHealth(); h.header.stamp = rospy.Time.now()
        h.valid = False; h.reason = 'planner session stopped'; pub.publish(h)
        terminate()
    return child.returncode or 0

if __name__ == '__main__':
    sys.exit(main())
