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

## @class GenericActionClient
## @brief The GenericActionClient
## launches a ROS node that will stream XML data from the machine tool and will launch a ROS
## action client if the machine tool requests an action.  For example, if the machine tool 
## requests a MaterialLoad action, it will change the MaterialLoad Event tag from 'READY' to
## 'ACTIVE'.  Once this change is captured by this class, the class launches a ROS MaterialLoadAction
## client.  Subsequently, the MaterialLoadAction server must be executed to process the action.
## If not, the system waits until the MaterialLoadAction server is available.
##
## Actions and goals are specified by a configuration file that must be included with the
## main program during execution. If this file is not provided, the node will terminate
## with an error message indicating the need for this file.
##
## Command line example:
##
##     bridge_client.py -i bridge_client_config.yaml
##     bridge_client.py -input bridge_client_config.yaml
##
## The class contains the following methods:
## setup_topic_data -- utilizes introspection to set up class instance variables.
## action_client -- function triggered by the xml_callback that executes a ROS action.
## xml_callback -- parses xml stream and launches action client.  Completes 'READY' handshake with machine tool.
class GenericActionClient():
    ## @brief Constructor for a GenericActionClient
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
        self.package = None
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

    ## @brief This function captures the topic package, type, action goals, and action
    ## state conversion from ROS to MTConnect as required for the ROS action client.
    ## This task is completed for each topic specified in the configuration file.
    ## 
    ## This function then performs a relative import of the topic via the getattr(import_module)
    ## function.  Data is stored in the following class attributes:
    ## 
    ##     self.lib_manifests --> used to track which manifests have been loaded.
    ##     self.type_handle   --> used for ROS SimpleActionClient, stores package name with action messages.
    ##     self.action_list   --> used for ROS SimpleActionClient, stores CNC action request strings.
    ##     self.action_goals  --> data structure for message class instances of the topic type.
    def setup_topic_data(self):
        for package, action in self.config.items():
            if package not in self.msg_parameters: # Only one ROS package in config by design
                # Load package manifest if unique
                if package not in self.lib_manifests:
                    roslib.load_manifest(package)
                    self.lib_manifests.append(package)

                # Import module
                rospy.loginfo('Importing --> ' + package + '.msg')
                self.type_handle = import_module(package + '.msg')
                
                # Capture package for action client
                self.package = package
                
                for action_req in action.keys():
                    # Capture action name and action goal xml tag for action client callback reference
                    goal_tag = self.config[package][action_req].keys()[0]
                    self.action_list[action_req] = {goal_tag : self.config[package][action_req][goal_tag].keys()}
                    
                    # Capture ROS Action parameters
                    self.action_goals[action_req] = None
        return    
    
    ## @brief ROS Action client function that compiles and sends the action goal to the
    ## dedicated ROS action server.  The function will block until the server becomes available.
    ## After the goal result is received from the dedicated ROS action server, the function
    ## sets the robot data item via the MTConnect adapter.
    ## 
    ## @param cb_data: tuple containing the following parameters:
    ## @param name: string containing the data item name for the robot action
    ## @param goals: dictionary of data_item:goal pairs in str:str or str:float format
    ## @param handle: Python <module 'mtconnect_msgs.msg'>
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
        
        # Check to make sure the action goal has been set
        if self.action_goals[name] == None:
            # Ping the current url for the goal Event
            goal_tag = self.action_list[name].keys()[0]
            response = bridge_library.xml_get_response((self.url, self.url_port, self.port, self.conn, 
                                                        self.mtool + "/current?path=//DataItem[@type='" + goal_tag.upper() +"']"))
            body = response.read()
            root = ElementTree.fromstring(body)

            # Set the action goals
            self.action_goals = bridge_library.set_goal(name, self.action_list[name], root, self.ns, self.action_goals)
            
        # Check goal source for required attributes
        for attrib, attrib_type, action_goal in zip(goal_handle.__slots__, goal_handle._slot_types, self.action_goals[name]):
            if bridge_library.type_check(attrib_type, action_goal) == False:
                rospy.logerr('INCOMPATIBLE GOAL TYPES ROS MSG: %s --> XML: %s' % (attrib_type, type(action_goal)))
                sys.exit()
        
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
        
        # Obtain result state
        result = client.get_state()
        
        # Prints out the result of the executing action
        rospy.loginfo('Returning the result --> %s' % result)
        
        # Set the Robot XML data item
        data_item = bridge_library.split_event(name)
        
        # Submit converted result to host via MTConnect adapter
        if result == 3: # SUCCEEDED from actionlib_msgs.msg.GoalStatus
            rospy.loginfo('Sending COMPLETE flag')
            bridge_library.action_cb((self.adapter, self.di_dict, data_item, 'COMPLETE'))
        return

    ## @brief Callback function that launches a ROS action client if the machine
    ## tool data item tag is 'ACTIVE'.  Once the action is completed, it completes
    ## the handshake signal between the machine tool and the robot via the MTConnect
    ## adapter.
    ## 
    ## @param chunk: xml data, read from response.read()
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
                    # Remove XML namespace string from the element tag for hash tables
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
		