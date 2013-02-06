#! /usr/bin/env python

import sys
import os

# Import custom Python modules for MTConnect Adapter interface
path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from mtconnect_adapter import Adapter

#from httplib import HTTPConnection
from xml.etree import ElementTree
import threading
import time
import urllib
import httplib
import socket

import read_config_file
import roslib
roslib.load_manifest('mtconnect_msgs')
import rospy
import mtconnect_msgs.msg

import closechuck
import closedoor
import opendoor
import openchuck

"""
assert ntries >= 1
for _ in range(ntries):
    try:
        page = urlopen(request, timeout=timeout)
        break # success
    except URLError as err:
        if not isinstance(err.reason, socket.timeout):
           raise # propagate non-timeout errors
else: # all ntries failed 
    raise err # re-raise the last timeout error
# use page here
"""

def check_connectivity(request, timeout):
    try:
        page = urlopen(request, timeout=timeout)
        return True
    except urllib.request.URLError as err:
        if not isinstance(err.reason, socket.timeout):
           raise # propagate non-timeout errors
        return False


#def check_connectivity(reference):
#    try:
#        urllib.request.urlopen(reference, timeout=1)
#        return True
#    except urllib.request.URLError:
#        return False
    
def talker():
    rospy.loginfo('Starting ROS Robot State Publisher Thread')
    pub1 = rospy.Publisher('RobotStateTopic', mtconnect_msgs.msg.RobotStates)
    pub2 = rospy.Publisher('RobotSpindleTopic', mtconnect_msgs.msg.RobotSpindle)
    #rospy.init_node('robot_link')
    msg1 = mtconnect_msgs.msg.RobotStates()
    msg2 = mtconnect_msgs.msg.RobotSpindle()
    
    while not rospy.is_shutdown():
        msg1.header.stamp = rospy.Time.now()
        msg1.avail.val = 1 # Available
        msg1.mode.val = 2 # Automatic
        msg1.rexec.val = 1 # Active
        
        msg2.c_unclamp.val = 1
        msg2.s_inter.val = 1

        pub1.publish(msg1)
        pub2.publish(msg2)
    return

def run_actions():
    rospy.loginfo('Starting Action Client Thread')
    cnc_conn = httplib.HTTPConnection('localhost', 5000)
    rbt_conn = httplib.HTTPConnection('localhost', 5001)
    ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')
    
    while True:
        # Update current XML tag states
        response = xml_get_response(cnc_conn, "/cnc/current")
        body = response.read()
        root = ElementTree.fromstring(body)
        cnc_ml = root.findall('.//m:MaterialLoad', namespaces = ns)
        cnc_mul = root.findall('.//m:MaterialUnload', namespaces = ns)
        
        response = xml_get_response(rbt_conn, "/Robot/current")
        body = response.read()
        root = ElementTree.fromstring(body)
        rbt_ml = root.findall('.//m:MaterialLoad', namespaces = ns)
        rbt_mul = root.findall('.//m:MaterialUnload', namespaces = ns)
        
        # If material load or material unload is ACTIVE, start chuck & door sequence
        if cnc_ml[0].text == 'ACTIVE' and rbt_ml[0].text == 'ACTIVE':
            # launch close chuck action client
            closechuck.close_chuck_client()
            rospy.loginfo('CloseChuck Action Completed')
            # launch close door action client
            closedoor.close_door_client()
            rospy.loginfo('CloseDoor Action Completed')
        elif cnc_mul[0].text == 'ACTIVE' and rbt_mul[0].text == 'ACTIVE':
            # launch open door action client
            opendoor.open_door_client()
            rospy.loginfo('OpenDoor Action Completed')
            # launch open chuck action client
            openchuck.open_chuck_client()
            rospy.loginfo('OpenChuck Action Completed')
        time.sleep(3)
    return

def xml_get_response(conn, req):
    conn.request("GET", req)
    response = conn.getresponse()
    if response.status != 200:
        rospy.loginfo("Request failed: %s - %d" % (response.reason, response.status))
        sys.exit(0)
    return response


def robot_init():
    adapter = Adapter(('0.0.0.0', 7900))

    # Start the adapter
    rospy.loginfo('Start the Robot Link adapter')
    adapter.start()
    
#    material_load_di = Event('material_load')
#    adapter.add_data_item(material_load_di)

#    material_unload_di = Event('material_unload')
#    adapter.add_data_item(material_unload_di)

    open_door_di = Event('open_door')
    adapter.add_data_item(open_door_di)

    close_door_di = Event('close_door')
    adapter.add_data_item(close_door_di)

    open_chuck_di = Event('open_chuck')
    adapter.add_data_item(open_chuck_di)

    close_chuck_di = Event('close_chuck')
    adapter.add_data_item(close_chuck_di)

    adapter.begin_gather()
    open_door_di.set_value('READY')
    close_door_di.set_value('READY')
    open_chuck_di.set_value('READY')
    close_chuck_di.set_value('READY')

#    material_load_di.set_value('READY')
#    material_unload_di.set_value('READY')

    adapter.complete_gather()
    return

    
if __name__ == '__main__':
    try:
        rospy.loginfo('Launching robot_link node, CTRL-C to terminate')
        robot_init()
        rospy.init_node('robot_link')
        
        #while True:
        #    try:
        #        if check_connectivity('localhost:5000', 2) and check_connectivity('localhost:5001', 2):
        #            break
        #    except socket.error as e:
        #        rospy.loginfo('HTTP connection error %s' % e)
        #        time.sleep(5)
        
        t1 = threading.Thread(target = talker)
        t1.start()
        t2 = threading.Thread(target = run_actions)
        t2.start()
    except rospy.ROSInterruptException:
        t1.join()
        t2.join()
