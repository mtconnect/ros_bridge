#!/usr/bin/env python

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
import optparse
import yaml
import operator
import thread
import re
import time
import socket
import urllib2
from importlib import import_module
from httplib import HTTPConnection
from xml.etree import ElementTree

# Import custom ROS-MTConnect function library
import bridge_library

# Import custom Python modules for MTConnect Adapter interface
path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from mtconnect_adapter import Adapter
from long_pull import LongPull

# Import ROS Python modules
import roslib
import rospy
import actionlib

class GenericActionClient():
    def __init__(self):
        # Initialize ROS generic client node
        rospy.init_node('ActionClient')
        
        # Setup MTConnect to ROS Conversion
        self.config = bridge_library.obtain_dataMap()
        self.msg_parameters = ['url', 'url_port', 'machine_tool', 'xml_namespace', 'adapter_port']
        self.url = self.config[self.msg_parameters[0]]
        self.url_port = self.config[self.msg_parameters[1]]
        self.mtool = self.config[self.msg_parameters[2]]
        self.ns = dict(m = self.config[self.msg_parameters[3]])
        self.port = self.config[self.msg_parameters[4]]
        
        # Check for url connectivity, dwell until system timeout
        bridge_library.check_connectivity((1,self.url,self.url_port))
        
        # Setup MTConnect Adapter for robot status data items
        self.adapter = Adapter((self.url, self.port))
        
        # Create empty lists for actions, message type handles, etc.
        self.lib_manifests = []
        self.type_handle = None
        self.action_list = {}
        self.action_goals = {}            
        self.action_conv = []
        self.namespace = None
        self.xml_goal = None
        self.di_dict = {} # XML data item dictionary to store MTConnect Adapter Events
        self.handshake = None
        
        # Setup action client data per the config file
        self.setup_topic_data()
        
        # Add data items to the MTConnect Adapter - must be unique data items
        bridge_library.add_event((self.adapter, self.action_list, self.di_dict, True))
        
        # Start the adapter
        rospy.loginfo('Start the Robot Link adapter')
        self.adapter.start()
        
        # Establish XML connection, read in current XML
        while True:
            try:
                self.conn = HTTPConnection(self.url, self.url_port)
                response = bridge_library.xml_get_response((self.url, self.url_port, self.port, self.conn, self.mtool + "/current"))
                body = response.read()
                break
            except socket.error as e:
                rospy.loginfo('HTTP connection error %s' % e)
                time.sleep(10)
        
        # Parse the XML and determine the current sequence and XML Event elements
        seq, elements = bridge_library.xml_components(body, self.ns, self.action_list)

        # Start a streaming XML connection
        response = bridge_library.xml_get_response((self.url, self.url_port, self.port, self.conn, self.mtool + "/sample?interval=1000&count=1000&from=" + seq))
        
        # Create class lock
        self.lock = thread.allocate_lock()

        # Create XML polling thread
        lp = LongPull(response)
        lp.long_pull(self.xml_callback) # Runs until user interrupts
    
    def setup_topic_data(self):
        """This function captures the topic namespace, type, action goals,
        and action state conversion from ROS to MTConnect as required for
        the ROS action client.  This task completed for each topic specified
        in the configuration file.
        
        This function then performs a relative import of the topic
        via the getattr(import_module) function.  Data is stored in the
        following class attributes:
        
            self.lib_manifests --> used to track which manifests have been loaded
            self.type_handle   --> used for ROS SimpleActionClient, stores namespace with action messages
            self.action_list   --> used for ROS SimpleActionClient, stores CNC action request strings
            self.action_goals  --> data structure for message class instances of the topic type
        """
        
        for namespace, action in self.config.items():
            if namespace not in self.msg_parameters: # Only one ROS namespace in config by design
                # Load package manifest if unique
                if namespace not in self.lib_manifests:
                    roslib.load_manifest(namespace)
                    self.lib_manifests.append(namespace)

                # Import module
                rospy.loginfo('Importing --> ' + namespace + '.msg')
                self.type_handle = import_module(namespace + '.msg')
                
                # Capture namespace for action client
                self.namespace = namespace
                
                for action_req in action.keys():
                    # Capture action name and action goal xml tag for action client callback reference
                    goal_tag = self.config[namespace][action_req].keys()[0]
                    self.action_list[action_req] = {goal_tag : self.config[namespace][action_req][goal_tag].keys()}
                    
                    # Capture ROS Action parameters
                    self.action_goals[action_req] = None
                    
                    # Capture ROS to MTConnect state conversion dictionary
                    #self.action_conv[action_req] = self.config[namespace][action_req]['conversion']
        return    
    
    def action_client(self, cb_data):
        # Execute ROS Action
        rospy.init_node('ActionClient')
        
        # Unpack action client data
        name, goals, handle = cb_data
        rospy.loginfo('Launched %s Action CLient' % name)
        
        # Creates the SimpleActionClient, passing the type of the action (MaterialLoadAction) to the constructor.
        # i.e. name = 'MaterialLoad', action_type = mtconnect_msgs.msg.MaterialLoadAction
        action_type = getattr(handle, name + 'Action')
        client = actionlib.SimpleActionClient(name + 'Client', action_type)
        
        # Waits until the action server has started up and started listening for goals.
        rospy.loginfo('Waiting for %s Dedicated Action Server' % name)
        client.wait_for_server()
        rospy.loginfo('%s Dedicated Action Server Activated' % name)

        # Creates a MaterialLoad goal to send to the action server.
        # i.e. goal = mtconnect_msgs.msg.MaterialLoad()
        goal_handle = getattr(handle, name + 'Goal')
        goal = goal_handle()
        
        # Check goal source for required attributes
        for attrib, attrib_type, action_goal in zip(goal_handle.__slots__, goal_handle._slot_types, self.action_goals[name]):
            rospy.loginfo('SLOT --> %s\tTYPE --> %s\tGOAL --> %s' % (attrib, attrib_type, action_goal))
            if attrib_type != type(action_goal):
                rospy.logerr('INCOMPATIBLE GOAL TYPES ROS MSG: %s --> XML: %s' % (attrib_type, type(action_goal)))
        
        # Set the goal attributes from XML data
        try:
            for attrib, xml_goal in zip(goal_handle.__slots__, self.action_goals[name]):
                setattr(goal, attrib, xml_goal)
        except Exception as e:
            rospy.logerr("ROS Action %s Client failed: %s" % (name, e))
        
        # Sends the goal to the action server.
        rospy.loginfo('Sending the goal')
        name_conv = bridge_library.split_event(name)
        client.send_goal(goal, done_cb = None, active_cb = bridge_library.action_cb((self.adapter, self.di_dict, name_conv, 'ACTIVE')))
        
        
        # Waits for the server to finish performing the action.
        rospy.loginfo('Waiting for result')
        client.wait_for_result()
        
        # Obtain result
        result = client.get_result() # result must be a string
        
        # Prints out the result of the executing action
        rospy.loginfo('Returning the result --> %s' % result)
        
        # Set the Robot XML data item
        data_item = bridge_library.split_event(name)
        
        # Obtain text string for result -- Simulation Only, Replace with ROS-MTConnect conversion
        di_result = getattr(result, result.__slots__[0])
        
        # Submit converted result to host via MTConnect adapter
        bridge_library.action_cb((self.adapter, self.di_dict, data_item, di_result))

        return

    def xml_callback(self, chunk):
        rospy.loginfo('*******************In PROCESS_XML callback***************')
        self.lock.acquire()
        try:
            # Only grab XML elements for CNC action requests 
            _, elements, self.action_goals = bridge_library.xml_components(chunk, self.ns, self.action_list, get_goal = True, action_goals = self.action_goals)

            if elements:
                # Check for existing handshake, reverse elements if necessary
                if self.handshake != elements[0].attrib['name']:
                    # Reverse element order
                    elements = elements[::-1]
                for e in elements:
                    # Remove namespace string from the element tag for hash tables
                    action_text = re.findall(r'(?<=\})\w+',e.tag)[0]
                    
                    # Check if CNC is requesting an action, if so, run action client
                    if e.text == 'ACTIVE':
                        self.action_client((action_text, self.action_goals[action_text], self.type_handle))
                        self.handshake = e.attrib['name']
                    # Check if CNC is submitting a handshake request
                    elif e.text == 'READY' and e.attrib['name'] == self.handshake:
                        # Send hand shake signal
                        bridge_library.action_cb((self.adapter, self.di_dict, e.attrib['name'], 'READY'))
                        self.handshake = None
        except Exception as e:
            rospy.logerr("Generic Action Client: Process XML callback failed: %s, releasing lock" % e)
        finally:
            self.lock.release()
        rospy.loginfo('*******************Done with PROCESS_XML callback***************')
        return

if __name__ == '__main__':
    try:
        rospy.loginfo('Starting Generic Action Client')
        robot_action = GenericActionClient()
    except rospy.ROSInterruptException:
        rospy.loginfo('program interrupted before completion')
		