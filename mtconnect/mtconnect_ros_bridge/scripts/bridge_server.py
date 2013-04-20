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
import urllib2
from Queue import Queue
from threading import Thread, Timer
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

## @class GenericActionServer
## @brief The GenericActionServer
## will launch a ROS node that blocks until a dedicated action request is made by the robot,
## such as 'OpenDoor'.  Once a request is received, the callback function will monitor
## the machine tool and complete the 'READY' handshake before returning the request result
## to the dedicated action client.
##
## Dedicated action client data items are specified by a configuration file that must be included with the
## main program during execution. If this file is not provided, the node will terminate
## with an error message indicating the need for this file.
##
## Command line example:
##
##     bridge_server.py -i bridge_server_config.yaml
##     bridge_server.py -input bridge_server_config.yaml
##
## The class contains the following methods:
## setup_topic_data -- sets up class instance variables.
## execute_cb -- triggered by a ROS action client, monitors machine tool response sequence and submits 'READY' handshake.
## xml_callback -- parses xml stream and stores xml into a data queue.
class GenericActionServer():
    ## @brief Constructor for a GenericActionServer
    def __init__(self):
        # Initialize ROS generic client node
        rospy.init_node('ActionServer')
        
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
        self.capture_xml = False
        self.di_dict = {} # XML data item dictionary to store MTConnect Adapter Events
        
        # Create a dictionary of _results to return upon action success
        self._resultDict = {}
        
        # Create a dictionary of action servers
        self._as = {}
        
        # Create a dictionary of server names
        self.server_name = {}
        
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
        
        # Create queue for streaming XML
        self.XML_queue = Queue()
        
        # Create XML polling thread
        lp = LongPull(response)
        xml_thread = Thread(target = lp.long_pull, args = (self.xml_callback,))
        xml_thread.daemon = True
        xml_thread.start()
    
    ## @brief This function imports the topic package and creates an action server for each
    ## package specified in the configuration file.
    ## 
    ## Data is stored in the following class attributes:
    ## 
    ##    self.lib_manifests --> used to track which manifests have been loaded
    ##    self.type_handle   --> used for ROS SimpleActionClient, stores package name with action messages
    ##    self.action_list   --> used for ROS SimpleActionClient, stores CNC action request strings
    ##    self.server_name   --> dictionary of data_item:DataItem pairs in str:str format
    ##    self._resultDict   --> dictionary of result class instances, i.e. {'MaterialLoad':mtconnect_msgs.msg._MaterialLoadResult.MaterialLoadResult()} 
    ##    self._as           --> dictionary of ROS action servers referenced by DataItem
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
                
                # Iterate through requests and create action result class instances and action servers
                for request in action:
                    # Capture action name for action client callback reference
                    self.action_list[request] = request
                    
                    # Store the ROS server name in a hash table
                    di_conv = bridge_library.split_event(request)
                    self.server_name[di_conv] = request
                    
                    # Create a dictionary of result class instances
                    request_class = getattr(self.type_handle, request + 'Result')
                    self._resultDict[request] = request_class()
                    
                    # Create instance of the action class
                    action_class = getattr(self.type_handle, request + 'Action')
                    
                    # Create action server for requested action, store in a hash table
                    self._as[request] = actionlib.SimpleActionServer(request + 'Client', action_class, self.execute_cb, False)
                    
                    # Start the action server
                    self._as[request].start()
        return
    
    ## @brief Callback function that monitors the machine tool action sequence for 'ACTIVE',
    ## 'COMPLETE', and 'READY' data item states.  Once the machine tool is completed with the
    ## requested action and sends the machine tool 'READY' signal, the ROS action server
    ## returns set_succeeded to the ROS action client that requested the action.
    ## 
    ## @param goal: from action client, class instance of the data item action goal, i.e. goal = mtconnect_msgs.msg.OpenDoorGoal()
    def execute_cb(self, goal):
        # If goal is OpenDoor, use the goal defined in the message as your key, access it as goal.__slots__[0]
        action = goal.__slots__[0]
        
        # Empty function -- assumes action was successful
        rospy.loginfo('In %s Callback -- determining action request result.' % self.server_name[action])
        
        # Check to make sure machine tool is READY, if not, ABORT the robot action request
        
        # Change to XML Tag format
        tokens = re.findall(r'([A-Z][a-z]*)', self.server_name[action])
        tokenlist = [val.upper() for val in tokens]
        di_tag = tokenlist[0] + '_' + tokenlist[1]
        
        # Check current status of XML Tag
        check_response = bridge_library.xml_get_response((self.url, self.url_port, self.port, self.conn, self.mtool + "/current?path=//DataItem[@type='" + di_tag + "']"))
        body = check_response.read()
        _, element = bridge_library.xml_components(body, self.ns, {self.server_name[action]:self.server_name[action]})
        
        # Complete the action request
        if element[0].text == 'READY': # Execute the action request
            # Submit goal value to MTConnect via agent (i.e. goal.open_door = 'ACTIVE')
            bridge_library.action_cb((self.adapter, self.di_dict, action, 'ACTIVE'))
            
            robot_hold = 0 # Used to make sure you only change robot state once after 'COMPLETE' received from CNC
            
            # Start capturing XML
            self.capture_xml = True
            
            # While loop to poll CNC XML until READY is received for Robot action request
            dwell = True
            while dwell == True:
                try:
                    # Obtain XML chunk
                    if not self.XML_queue.empty():
                        chunk = self.XML_queue.get()
                        
                        # Parse the XML and determine the current sequence and XML Event elements
                        root = ElementTree.fromstring(chunk)
                        element_list = root.findall('.//m:' + self.server_name[action], namespaces = self.ns)
                        if len(element_list) > 1:
                            rospy.logdebug('XML --> %s' % chunk)
                        if element_list:
                            # Must iterate -- multiple elements possible for a single tag
                            for element in element_list:
                                if element.text == 'ACTIVE':
                                    # Set accepted back to action client
                                    pass
                                
                                # While polling monitor CNC response for COMPLETE, submit READY handshake
                                elif element.text == 'COMPLETE' and robot_hold == 0:
                                    bridge_library.action_cb((self.adapter, self.di_dict, action, 'READY'))
                                    robot_hold = 1
                                
                                elif element.text == 'READY' and robot_hold == 1:
                                    dwell = False
                                    self.capture_xml = False
                                    
                                    # When response is READY, set server result and communicate as below:
                                    # Extract action attribute
                                    result_attribute = self._resultDict[self.server_name[action]].__slots__[0]
                                    
                                    # Set the attribute per the ROS to MTConnect conversion
                                    setattr(self._resultDict[self.server_name[action]], result_attribute, 'READY')
                                    
                                    # Indicate a successful action
                                    self._as[self.server_name[action]].set_succeeded(self._resultDict[self.server_name[action]])
                                    rospy.loginfo('In %s Callback -- action succeeded.' % self.server_name[action])
                                
                                elif element.text == 'FAIL':
                                    bridge_library.action_cb((self.adapter, self.di_dict, action, 'FAIL'))
                                    dwell = False
                                    self.capture_xml = False
                                    
                                    # When response is FAIL, set server result and communicate as below:
                                    # Extract action attribute.  For CNC Actions, only one result --> i.e. OpenDoorResult.__slots__[0] = 'door_ready'
                                    result_attribute = self._resultDict[self.server_name[action]].__slots__[0]
                                    
                                    # Set the attribute per the ROS to MTConnect conversion
                                    setattr(self._resultDict[self.server_name[action]], result_attribute, 'FAIL')
                                    
                                    # Indicate an aborted action
                                    self._as[self.server_name[action]].set_aborted(self._resultDict[self.server_name[action]])
                                    rospy.loginfo('In %s Callback -- action aborted by CNC.' % self.server_name[action])
                        
                        # Release the queue
                        self.XML_queue.task_done()
                except rospy.ROSInterruptException:
                    rospy.loginfo('program interrupted before completion')
        else: # Abort the action request, machine tool not ready or faulted
            # Extract action attribute.  For CNC Actions, only one result --> i.e. OpenDoorResult.__slots__[0] = 'door_ready'
            result_attribute = self._resultDict[self.server_name[action]].__slots__[0]
            
            # Set the attribute per the ROS to MTConnect conversion
            setattr(self._resultDict[self.server_name[action]], result_attribute, 'NOT_READY')
            
            # Indicate an aborted action
            self._as[self.server_name[action]].set_aborted(self._resultDict[self.server_name[action]])
            rospy.loginfo('In %s Callback -- action aborted, machine tool NOT_READY, FAULTED, or UNAVAILABLE.' % self.server_name[action])
        return

    ## @brief Processes the xml chunk provided by the LongPull class instance and stores the result
    ## into the XML_queue since the tags have changed.  The queue is used to ensure capture of updated
    ## tags while the execute_cb function is being executed.
    ## @param chunk: xml data, read from response.read()
    def xml_callback(self, chunk):
        #rospy.loginfo('*******************In PROCESS_XML callback***************')
        try:
            if self.capture_xml == True:
                self.XML_queue.put(chunk)
                if self.XML_queue.qsize() > 1:
                    rospy.logdebug('STORED XML INTO QUEUE, WAITING ON ROS ACTION SERVER, QUEUE SIZE --> %s' % self.XML_queue.qsize())
        except Exception as e:
            rospy.logerr("Bridge Server: Process XML callback failed: %s, releasing lock" % e)
        finally:
            pass
        #rospy.loginfo('*******************Done with PROCESS_XML callback***************')
        return

if __name__ == '__main__':
    try:
        rospy.loginfo('Started Generic Action Server')
        robot_action = GenericActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('program interrupted before completion')
        xml_thread.join()
