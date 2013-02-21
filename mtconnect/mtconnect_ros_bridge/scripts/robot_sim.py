#! /usr/bin/env python

"""
   Copyright 2013 Southwest Research Institute
 
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
 
     http://www.apache.org/licenses/LICENSE-2.0
 
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   """

# Import standard Python modules
import sys
import os
from Queue import Queue
from threading import Thread, Timer
from xml.etree import ElementTree
import threading
import time
import urllib2
import httplib
import socket

# Import custom ROS-MTConnect function library
import bridge_library

# Import custom Python modules for MTConnect Adapter interface
path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')
from long_pull import LongPull

# Import ROS Python Modules, and ROS mtconnect_msgs package
import roslib
roslib.load_manifest('mtconnect_msgs')
import rospy
import mtconnect_msgs.msg

# Import Custom Action Clients
import closechuck
import closedoor
import opendoor
import openchuck

class RobotSim():
    def __init__(self):
        # Initialize ROS generic client node
        rospy.init_node('robot_link')
        
        # Check for url connectivity, dwell until system timeout
        bridge_library.check_connectivity((1, 'localhost', 5000))
        bridge_library.check_connectivity((1, 'localhost', 5001))
        
        # Global variables
        self.XML_CNC_queue = Queue()
        self.XML_RBT_queue = Queue()
        self.cnc_capture_xml = True
        self.rbt_capture_xml = True
        self.cnc_conn = httplib.HTTPConnection('localhost', 5000)
        self.rbt_conn = httplib.HTTPConnection('localhost', 5001)
        self.ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')
        self.data_items = {'MaterialLoad' : 'MaterialLoad', 'MaterialUnload' : 'MaterialUnload'}
        
        # Establish CNC XML connection, read in current XML
        cnc_response = bridge_library.xml_get_response(('localhost', 5000, None, self.cnc_conn, "/cnc/current"))
        cnc_body = cnc_response.read()
        
        # Parse the XML and determine the current sequence and XML Event elements
        seq, elements = bridge_library.xml_components(cnc_body, self.ns, self.data_items)
        
        # Start a streaming XML connection
        cnc_response = bridge_library.xml_get_response(('localhost', 5000, None, self.cnc_conn, "/cnc/sample?interval=1000&count=1000&from=" + seq))
        
        # Create CNC XML polling thread
        cnc_lp = LongPull(cnc_response)
        self.cnc_thread = threading.Thread(target = cnc_lp.long_pull, args = (self.cnc_xml_callback,))
        self.cnc_thread.daemon = True
        self.cnc_thread.start()
        rospy.loginfo('STARTED STREAMING CNC XML THREAD')
        
        # Establish Robot XML connection, read in current XML
        rbt_response = bridge_library.xml_get_response(('localhost', 5001, None, self.rbt_conn, "/Robot/current"))
        rbt_body = rbt_response.read()
        
        # Parse the XML and determine the current sequence and XML Event elements
        seq, elements = bridge_library.xml_components(rbt_body, self.ns, self.data_items)
        
        # Start a streaming XML connection
        rbt_response = bridge_library.xml_get_response(('localhost', 5001, None, self.rbt_conn, "/Robot/sample?interval=1000&count=1000&from=" + seq))
        
        # Create Robot XML polling thread
        rbt_lp = LongPull(rbt_response)
        self.rbt_thread = threading.Thread(target = rbt_lp.long_pull, args = (self.rbt_xml_callback,))
        self.rbt_thread.daemon = True
        self.rbt_thread.start()
        rospy.loginfo('STARTED STREAMING ROBOT XML THREAD')
        
        # Create robot initialization publisher thread
        self.t1 = threading.Thread(target = self.talker)
        self.t1.daemon = True
        self.t1.start()
        rospy.loginfo('STARTED ROBOT INITIALIZATION THREAD')
        
        # Create state sequence thread
        self.t2 = threading.Thread(target = self.run_actions)
        self.t2.daemon = True
        self.t2.start()
        rospy.loginfo('STARTED STATE MACHINE THREAD')

    def stop_thread(self):
        self.t1.join()
        self.t2.join()
        self.cnc_thread.join()
        self.rbt_thread.join()
        return
 
    def talker(self):
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

    def run_actions(self):
        rospy.loginfo('Starting Robot State Machine Thread')
        cnc_state = None
        robot_state = None
        sequence = None
        
        while True:
            # Determine if CNC is in MaterialLoad or MaterialUnload sequence
            if not self.XML_CNC_queue.empty():
                cnc_body = self.XML_CNC_queue.get()
                cnc_root = ElementTree.fromstring(cnc_body)
                tokenML = cnc_root.findall('.//m:MaterialLoad', namespaces = self.ns)
                tokenMUL = cnc_root.findall('.//m:MaterialUnload', namespaces = self.ns)
                
                if tokenML and tokenML[0].text == 'ACTIVE':
                    if sequence is None:
                        sequence = 'MaterialLoad'
                    cnc_state = tokenML[0].text
                    self.cnc_capture_xml = False
                    rospy.loginfo('CNC XML %s --> %s' % (sequence, tokenML[0].text))
                elif tokenMUL and tokenMUL[0].text == 'ACTIVE':
                    if sequence is None:
                        sequence = 'MaterialUnload'
                    cnc_state = tokenMUL[0].text
                    self.cnc_capture_xml = False
                    rospy.loginfo('CNC XML %s --> %s' % (sequence, tokenMUL[0].text))
                
                # Release the queue
                self.XML_CNC_queue.task_done()
                
            
            # Determine if the ROBOT has acknowledged the MaterialLoad or MaterialUnload sequence
            if not self.XML_RBT_queue.empty():
                rbt_body = self.XML_RBT_queue.get()
                rbt_root = ElementTree.fromstring(rbt_body)
                tokenML = rbt_root.findall('.//m:MaterialLoad', namespaces = self.ns)
                tokenMUL = rbt_root.findall('.//m:MaterialUnload', namespaces = self.ns)
                
                if tokenML and tokenML[0].text == 'ACTIVE':
                    if sequence is None:
                        sequence = 'MaterialLoad'
                    robot_state = tokenML[0].text
                    self.rbt_capture_xml = False
                    rospy.loginfo('ROBOT XML %s --> %s' % (sequence, tokenML[0].text))
                elif tokenMUL and tokenMUL[0].text == 'ACTIVE':
                    if sequence is None:
                        sequence = 'MaterialUnload'
                    robot_state = tokenMUL[0].text
                    self.rbt_capture_xml = False
                    rospy.loginfo('ROBOT XML %s --> %s' % (sequence, tokenMUL[0].text))
                
                self.XML_RBT_queue.task_done()
            
            # If material load for both devices is ACTIVE, start chuck & door close sequence
            if cnc_state == 'ACTIVE' and robot_state == 'ACTIVE' and sequence == 'MaterialLoad':
                rospy.loginfo('Starting Action Clients required by Material Load')
                
                # launch close chuck action client
                #opendoor.open_door_client()
                #rospy.loginfo('OpenDoor Action Completed')
                closechuck.close_chuck_client()
                rospy.loginfo('CloseChuck Action Completed')
                
                # start capturing XML again
                self.cnc_capture_xml = True
                self.rbt_capture_xml = True
                
                # launch close door action client
                closedoor.close_door_client()
                rospy.loginfo('CloseDoor Action Completed')
                
                # reset state variables
                cnc_state = None
                robot_state = None
                sequence = None
            
            # If material unload for both devices is ACTIVE, start chuck & door open sequence
            if cnc_state == 'ACTIVE' and robot_state == 'ACTIVE' and sequence == 'MaterialUnload':
                rospy.loginfo('Starting Action Clients required by Material Unload')
                
                # launch open door action client
                opendoor.open_door_client()
                rospy.loginfo('OpenDoor Action Completed')
                
                # start capturing XML again
                self.cnc_capture_xml = True
                self.rbt_capture_xml = True
                
                # launch open chuck action client
                openchuck.open_chuck_client()
                rospy.loginfo('OpenChuck Action Completed')
                
                # reset state variables
                cnc_state = None
                robot_state = None
                sequence = None
                
                # allow time for CNC state machine to initialize
                time.sleep(5)
        return

    def cnc_xml_callback(self, chunk):
        #rospy.loginfo('*******************In CNC XML Queue callback***************')
        if self.cnc_capture_xml == True:
            try:
                self.XML_CNC_queue.put(chunk)
                rospy.loginfo('PUTTING CNC XML INTO QUEUE. NUMBER OF QUEUED OBJECTS %s' % self.XML_CNC_queue.qsize())
                if self.XML_CNC_queue.qsize() > 1:
                    rospy.loginfo('STORED CNC XML INTO QUEUE, WAITING ON ROS ACTION SERVER, QUEUE SIZE --> %s' % self.XML_CNC_queue.qsize())
            except Exception as e:
                rospy.logerr("Robot Simulation: CNC XML Queue callback failed: %s, releasing lock" % e)
            finally:
                pass
        #rospy.loginfo('*******************Done with CNC XML Queue callback***************')
        return
    
    def rbt_xml_callback(self, chunk):
        #rospy.loginfo('*******************In ROBOT XML Queue callback***************')
        if self.rbt_capture_xml == True:
            try:
                self.XML_RBT_queue.put(chunk)
                rospy.loginfo('PUTTING ROBOT XML INTO QUEUE. NUMBER OF QUEUED OBJECTS %s' % self.XML_RBT_queue.qsize())
                if self.XML_RBT_queue.qsize() > 1:
                    rospy.loginfo('STORED ROBOT XML INTO QUEUE, WAITING ON ROS ACTION SERVER, QUEUE SIZE --> %s' % self.XML_RBT_queue.qsize())
            except Exception as e:
                rospy.logerr("Robot Simulation: ROBOT XML Queue callback failed: %s, releasing lock" % e)
            finally:
                pass
        #rospy.loginfo('*******************Done with ROBOT XML Queue callback***************')
        return

    
if __name__ == '__main__':
    try:
        rospy.loginfo('Launching robot_link node, CTRL-C to terminate')
        simulator = RobotSim()
        rospy.spin()
    except rospy.ROSInterruptException:
        RobotSim.stop_thread()
