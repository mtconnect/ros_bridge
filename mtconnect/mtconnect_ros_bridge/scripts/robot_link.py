#! /usr/bin/env python

import sys
import os

# Import custom Python modules for MTConnect Adapter interface
path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from mtconnect_adapter import Adapter
from long_pull import LongPull

#from httplib import HTTPConnection
from Queue import Queue
from threading import Thread, Timer
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

# Global variables
XML_CNC_queue = Queue()
XML_RBT_queue = Queue()
capture_xml = False
cnc_conn = httplib.HTTPConnection('localhost', 5000)
rbt_conn = httplib.HTTPConnection('localhost', 5001)
ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')

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
    rospy.loginfo('Starting Robot State Machine Thread')
    #cnc_conn = httplib.HTTPConnection('localhost', 5000)
    #rbt_conn = httplib.HTTPConnection('localhost', 5001)
    
    while True:
        capture_xml = True
        # Update current XML tag states
        #response = xml_get_response(cnc_conn, "/cnc/current")
        #body = response.read()
        #root = ElementTree.fromstring(body)
        #cnc_ml = root.findall('.//m:MaterialLoad', namespaces = ns)
        #cnc_mul = root.findall('.//m:MaterialUnload', namespaces = ns)
        
        #response = xml_get_response(rbt_conn, "/Robot/current")
        #body = response.read()
        #root = ElementTree.fromstring(body)
        #rbt_ml = root.findall('.//m:MaterialLoad', namespaces = ns)
        #rbt_mul = root.findall('.//m:MaterialUnload', namespaces = ns)
        
        if not XML_CNC_queue.empty() and not XML_RBT_queue.empty():
            cnc_body = XML_CNC_queue.get()
            cnc_root = ElementTree.fromstring(cnc_body)
            cnc_ml = cnc_root.findall('.//m:MaterialLoad', namespaces = ns)
            cnc_mul = cnc_root.findall('.//m:MaterialUnload', namespaces = ns)
            
            rbt_body = XML_RBT_queue.get()
            rbt_root = ElementTree.fromstring(rbt_body)
            rbt_ml = rbt_root.findall('.//m:MaterialLoad', namespaces = ns)
            rbt_mul = rbt_root.findall('.//m:MaterialUnload', namespaces = ns)
        
            # If material load or material unload is ACTIVE, start chuck & door sequence
            if cnc_ml[0].text == 'ACTIVE' and rbt_ml[0].text == 'ACTIVE':
                capture_xml = False
                # launch close chuck action client
                closechuck.close_chuck_client()
                rospy.loginfo('CloseChuck Action Completed')
                # launch close door action client
                closedoor.close_door_client()
                rospy.loginfo('CloseDoor Action Completed')
                time.sleep(6)
            elif cnc_mul[0].text == 'ACTIVE' and rbt_mul[0].text == 'ACTIVE':
                capture_xml = False
                # launch open door action client
                opendoor.open_door_client()
                rospy.loginfo('OpenDoor Action Completed')
                # launch open chuck action client
                openchuck.open_chuck_client()
                rospy.loginfo('OpenChuck Action Completed')
                time.sleep(5)
            
            XML_CNC_queue.task_done()
            XML_RBT_queue.task_done()
        
        time.sleep(0.5)
    return

def xml_get_response(conn, req):
    conn.request("GET", req)
    response = conn.getresponse()
    if response.status != 200:
        rospy.loginfo("Request failed: %s - %d" % (response.reason, response.status))
        sys.exit(0)
    return response

def xml_components(xml):
    """ Find all elements in the updated xml.  root.find requires namespaces to be a dictionary.
    Return sequence and elements to process ROS msgs.
    """
    root = ElementTree.fromstring(xml)
    header = root.find('.//m:Header', namespaces = ns)
    nextSeq = header.attrib['nextSequence']
    elements = root.findall('.//m:Events/*', namespaces = ns)        
    return nextSeq, elements

def robot_init():
    adapter = Adapter(('0.0.0.0', 7900))

    # Start the adapter
    rospy.loginfo('Start the Robot Link adapter')
    adapter.start()

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

    adapter.complete_gather()
    return

def cnc_xml_callback(chunk):
    #rospy.loginfo('*******************In PROCESS_XML callback***************')
    try:
        if capture_xml == True:
            XML_CNC_queue.put(chunk)
            #rospy.loginfo('PUTTING XML INTO QUEUE %s\tNUMBER OF QUEUED OBJECTS %s' % (XML_CNC_queue, XML_CNC_queue.qsize()))
            if XML_CNC_queue.qsize() > 1:
                rospy.loginfo('STORED XML INTO QUEUE, WAITING ON ROS ACTION SERVER, QUEUE SIZE --> %s' % XML_CNC_queue.qsize())
    except Exception as e:
        rospy.logerr("Bridge Server: Process XML callback failed: %s, releasing lock" % e)
    finally:
        pass
    #rospy.loginfo('*******************Done with PROCESS_XML callback***************')
    return

def rbt_xml_callback(chunk):
    try:
        if capture_xml == True:
            XML_RBT_queue.put(chunk)
            #rospy.loginfo('PUTTING XML INTO QUEUE %s\tNUMBER OF QUEUED OBJECTS %s' % (XML_RBT_queue, XML_RBT_queue.qsize()))
            if XML_RBT_queue.qsize() > 1:
                rospy.loginfo('STORED XML INTO QUEUE, WAITING ON ROS ACTION SERVER, QUEUE SIZE --> %s' % XML_RBT_queue.qsize())
    except Exception as e:
        rospy.logerr("Bridge Server: Process XML callback failed: %s, releasing lock" % e)
    finally:
        pass
    #rospy.loginfo('*******************Done with PROCESS_XML callback***************')
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
        
        
        # Establish CNC XML connection, read in current XML
        response = xml_get_response(cnc_conn, "/cnc/current")
        cnc_body = response.read()
        
        # Parse the XML and determine the current sequence and XML Event elements
        seq, elements = xml_components(cnc_body)
        
        # Start a streaming XML connection
        cnc_response = xml_get_response(cnc_conn, "/cnc/sample?interval=1000&count=1000&from=" + seq)
        
        # Create CNC XML polling thread
        lp = LongPull(cnc_response)
        cnc_thread = threading.Thread(target = lp.long_pull, args = (cnc_xml_callback,))
        cnc_thread.daemon = True
        cnc_thread.start()
        rospy.loginfo('STARTED STREAMING CNC XML THREAD')
        
        # Establish Robot XML connection, read in current XML
        rbt_response = xml_get_response(rbt_conn, "/Robot/current")
        rbt_body = rbt_response.read()
        
        # Parse the XML and determine the current sequence and XML Event elements
        seq, elements = xml_components(rbt_body)
        
        # Start a streaming XML connection
        rbt_response = xml_get_response(rbt_conn, "/Robot/sample?interval=1000&count=1000&from=" + seq)
        
        # Create Robot XML polling thread
        lp = LongPull(rbt_response)
        rbt_thread = threading.Thread(target = lp.long_pull, args = (rbt_xml_callback,))
        rbt_thread.daemon = True
        rbt_thread.start()
        rospy.loginfo('STARTED STREAMING ROBOT XML THREAD')
        
        t1 = threading.Thread(target = talker)
        t1.daemon = True
        t1.start()
        rospy.loginfo('STARTED ROBOT INITIALIZATION THREAD')
        
        t2 = threading.Thread(target = run_actions)
        t2.daemon = True
        t2.start()
        rospy.loginfo('STARTED STATE MACHINE THREAD')
    except rospy.ROSInterruptException:
        t1.join()
        t2.join()
        cnc_thread.join()
        rbt_thread.join()
