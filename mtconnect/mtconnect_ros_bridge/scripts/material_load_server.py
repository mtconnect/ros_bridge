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

from httplib import HTTPConnection
from xml.etree import ElementTree

import roslib; roslib.load_manifest('mtconnect_msgs')
import rospy
import actionlib
import mtconnect_msgs.msg


class MaterialLoadServer():
    """DOCSTRING
    """

    def __init__(self):
        # Setup MTConnect to ROS Conversion -- TBD
        #self.dataMap = read_config_file.obtain_dataMap()
        
        self.server_name = 'MaterialLoad'
        self._result = mtconnect_msgs.msg.MaterialLoadResult()
        self._as = actionlib.SimpleActionServer('MaterialLoadClient', mtconnect_msgs.msg.MaterialLoadAction, self.execute_cb, False)
        self._as.start()
        self._as.accept_new_goal()
    
    def execute_cb(self, goal):
        rospy.loginfo('In %s Bridge Server Callback -- determining action request result.' % self.server_name)
        
        # Required actions to complete requested action
        cnc_actions = {'CloseChuck':None, 'CloseDoor':None}
        
        # Establish XML connection, read in current XML
        conn = HTTPConnection('localhost', 5000)
        conn.request("GET", "/cnc/current")
        
        # Start while loop and check for cnc action changes
        dwell = True
        while dwell == True:
            response = conn.getresponse()
            if response.status != 200:
                rospy.loginfo("Request failed: %s - %d" % (response.reason, response.status))
                sys.exit(0)

            cnc_body = response.read()
            root = ElementTree.fromstring(cnc_body)
            ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')
            #header = root.find('.//m:Header', namespaces=ns)
            #nextSeq = header.attrib['nextSequence']
            elements = []
            for action in cnc_actions.keys():
                find_action = root.findall('.//m:' + action, namespaces=ns)
                if find_action: # Element list is not empty
                    elements.append(find_action[0])
        
            # Check that CNC completed actions
            for e, key in zip(elements, cnc_actions.keys()):
                if e.text == 'READY':
                    cnc_actions[key] = 'READY'

            if None not in cnc_actions.values():
                # Set action attribute -- empty function, assumes robot loaded material
                self._result.load_state = 'COMPLETE'
                dwell = False
        
        # Indicate a successful action
        self._as.set_succeeded(self._result)
        rospy.loginfo('In %s Callback -- action succeeded. Result --> %s' % (self.server_name, self._result.load_state))
        return

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('MaterialLoadServer')
    rospy.loginfo('Started ROS MaterialLoad Server')
    
    # Launch the action server
    server = MaterialLoadServer()
    rospy.spin()
