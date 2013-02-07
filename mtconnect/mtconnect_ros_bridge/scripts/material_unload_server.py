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

import logging
import threading
import time
import collections
from httplib import HTTPConnection
from xml.etree import ElementTree

import roslib; roslib.load_manifest('mtconnect_msgs')
import rospy
import actionlib
import mtconnect_msgs.msg


class MaterialUnloadServer():
    """DOCSTRING
    """

    def __init__(self):
        # Setup MTConnect to ROS Conversion -- TBD
        #self.config = read_config_file.obtain_dataMap()
        
        self.server_name = 'MaterialUnload'
        self.conn = conn = HTTPConnection('localhost', 5000)
        self.ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')
        self.sequence = {'ACTIVE':'COMPLETE', 'COMPLETE':'READY'}
        
        self._result = mtconnect_msgs.msg.MaterialUnloadResult()
        self._as = actionlib.SimpleActionServer('MaterialUnloadClient', mtconnect_msgs.msg.MaterialUnloadAction, self.execute_cb, False)
        self._as.start()
        self._as.accept_new_goal()
    
    def execute_cb(self, goal):
        rospy.loginfo('In %s Bridge Server Callback -- determining action request result.' % self.server_name)
        
        # Required actions to complete requested action
        cnc_actions = {'OpenChuck':None, 'OpenDoor':None}
        cnc_target = collections.OrderedDict()
        cnc_target['OpenChuck'] = ['COMPLETE', None]
        cnc_target['OpenDoor'] = ['COMPLETE', None]
        
        # Start while loop and check for cnc action changes
        dwell = True
        inloop = 0
        start = time.time()
        while dwell == True:
            # Establish XML connection, read in current XML
            self.conn.request("GET", "/cnc/current")
            response = self.conn.getresponse()
            if response.status != 200:
                rospy.loginfo("Request failed: %s - %d" % (response.reason, response.status))
                sys.exit(0)

            cnc_body = response.read()
            root = ElementTree.fromstring(cnc_body)
            cnc_target['OpenChuck'][1] = root.findall('.//m:OpenChuck', namespaces=self.ns)[0]
            cnc_target['OpenDoor'][1] = root.findall('.//m:OpenDoor', namespaces=self.ns)[0]
            #rospy.loginfo('OpenChuck --> %s\tOpenDoor --> %s' % (cnc_target['OpenChuck'][1].text, cnc_target['OpenDoor'][1].text))
            
            if inloop == 0:
                rospy.loginfo('In MaterialUnload Server while loop')
                rospy.loginfo('OpenChuck --> %s\tOpenDoor --> %s' % (cnc_target['OpenChuck'][1].text, cnc_target['OpenDoor'][1].text))
                inloop = 1
            
            # Wait for Close Chuck and Close Door Cycle
            for device, target in cnc_target.items():
                if target[0] != 'PASSED':
                    #rospy.loginfo('DEVICE --> %s\tTARGET --> %s' % (device, target[1].text))
                    #time.sleep(5)
                    if target[1].text == target[0] and target[0] != 'READY':
                        cnc_target[device][0] = self.sequence[target[1].text]
                        cnc_actions[device] = 'READY'
                        cnc_target[device][0] = 'PASSED'
                        rospy.loginfo('%s PASSED' % device)
                        rospy.loginfo('cnc_action_values --> %s' % cnc_actions.values())

            # Send the successful result    
            if None not in cnc_actions.values():
                # Set action attribute -- empty function, assumes robot loaded material
                self._result.unload_state = 'COMPLETE'
                dwell = False
            
            # Check for timeout
            if time.time() - start > 120.0:
                rospy.loginfo('Material Unload Server Timed Out')
                
        
        # Indicate a successful action
        self._as.set_succeeded(self._result)
        rospy.loginfo('In %s Callback -- action succeeded. Result --> %s' % (self.server_name, self._result.unload_state))
        return self._result

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('MaterialUnloadServer')
    rospy.loginfo('Started ROS MaterialUnload Server')
    
    # Launch the action server
    server = MaterialUnloadServer()
    rospy.spin()
