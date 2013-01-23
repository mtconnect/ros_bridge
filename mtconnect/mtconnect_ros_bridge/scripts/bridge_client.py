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
import operator
import thread
import re
import time
from importlib import import_module
from httplib import HTTPConnection
from xml.etree import ElementTree

# Import custom Python module to read config file
import read_config_file

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
        self.config = read_config_file.obtain_dataMap()
        
        # Setup MTConnect Adapter for robot status data items
        self.adapter = Adapter(('0.0.0.0', self.config['Port']))
        
        # Create empty lists for actions, message type handles, etc.
        self.lib_manifests = []
        self.type_handle = None
        self.action_list = {}
        self.action_goals = {}
        self.action_conv = []
        self.namespace = None
        self.xml_goal = None
        
        # Trigger to initiate ROS action callback
        self.trigger = None
        
        # Setup action client data per the config file
        self.setup_topic_data()
        
        # Add data items to the MTConnect Adapter - must be unique data items
        self.add_agent()
        
        # Start the adapter
        rospy.loginfo('Start the Robot Link adapter')
        self.adapter.start()
        
        # Establish XML connection, read in current XML
        conn = HTTPConnection('localhost', self.config['localhost'])
        response = self.xml_get_response(conn, "/cnc/current")
        cnc_body = response.read()
        
        # Parse the XML and determine the current sequence and XML Event elements
        seq, elements = self.xml_components(cnc_body)
        
        # Use XML to establish current data item state dictionary
        self.di_current = {re.findall(r'(?<=\})\w+',e.tag)[0]:e.text for e in elements}
        print 'self.di_current = %s' % self.di_current
        
        # Start a streaming XML connection
        response = self.xml_http_connection(conn, "/cnc/sample?interval=1000&count=1000&from=" + seq)
        
        # Create class lock
        self.lock = thread.allocate_lock()

        # Create ROS action thread
        #rospy.Timer(rospy.Duration(0.1), self.ros_action)
        
        # Create XML polling thread
        lp = LongPull(response)
        lp.long_pull(self.xml_callback) # Runs until user interrupts
    
    def add_agent(self):
        di = []
        for xml_tag in self.action_list:
            di.append(self.split_event(xml_tag))
            rospy.loginfo('data_item --> %s' % xml_tag)
        
        
        for data_item in di:
            co_str = "self." + data_item + "_di = Event('" + data_item + "')"
            co_exec = compile(co_str, '', 'exec')
            exec(co_exec)

            co_str = 'self.adapter.add_data_item(self.' + data_item + '_di)'
            co_exec = compile(co_str, '', 'exec')
            exec(co_exec)
        
        # Set initial states for robot actions (will be removed -- needed for simulation only)
        self.adapter.begin_gather()
        for data_item in di:
            co_str = "self." + data_item + "_di.set_value('READY')"
            co_exec = compile(co_str, '', 'exec')
            exec(co_exec)
        self.adapter.complete_gather()
        return
    
    def split_event(self, xml_tag):
        tokens = re.findall(r'([A-Z][a-z]*)', xml_tag)
        tokenlist = [val.lower() for val in tokens]
        data_item = tokenlist[0] + '_' + tokenlist[1]
        return data_item
                
    def setup_topic_data(self):
        """This function captures the topic namespace, type, action goals,
        and action state conversion from ROS to MTConnect as required for
        the ROS action client.  This task completed for each topic specified
        in the configuration file.
        
        This function then performs a relative import of the topic
        via the getattr(import_module) function.  Data is stored in the
        following class attributes:
        
            self.lib_manifests --> used to track which manifests have been loaded
            self.type_handle   --> used for ROS SimpleActionClient
            self.action_list   --> used for ROS SimpleActionClient
            self.action_goals  --> data structure for msg class instances of the topic type
        """
        
        for namespace, action in self.config.items():
            if namespace != 'localhost' and namespace != 'Port': # Only one ROS namespace in config by design
                # Load package manifest if unique
                if namespace not in self.lib_manifests:
                    roslib.load_manifest(namespace)
                    self.lib_manifests.append(namespace)
    
                # Import module
                rospy.loginfo('Importing --> ' + namespace + '.msg')
                #self.type_handle.append(import_module(namespace + '.msg'))
                self.type_handle = import_module(namespace + '.msg')
                
                # Capture namespace for action client
                self.namespace = namespace
                
                for action_req in action.keys():
                    # Capture action name for action client callback reference
                    #self.action_list.append(action_req)
                    self.action_list[action_req] = action_req
                    
                    # Capture ROS Action parameters
                    goal_key = self.config[namespace][action_req].keys()[0]
                    #self.action_goals.append(self.config[namespace][action_req][goal_key].keys())
                    self.action_goals[action_req] = self.config[namespace][action_req][goal_key].keys()
                    
                    # Capture ROS to MTConnect state conversion dictionary
                    #self.action_conv.append(self.config[namespace][action]['conversion'])
        return
    
    def xml_http_connection(self, conn, req):
        conn.request("GET", req)
        response = conn.getresponse()
        if response.status != 200:
            rospy.loginfo("Request failed: %s - %d" % (response.reason, response.status))
            sys.exit(0)
        else:
            rospy.loginfo('Request --> %s, Status --> %s' % (response.reason, response.status))
        return response
    
    def xml_get_response(self, conn, req):
        conn.request("GET", req)
        response = conn.getresponse()
        if response.status != 200:
            rospy.loginfo("Request failed: %s - %d" % (response.reason, response.status))
            sys.exit(0)
        else:
            rospy.loginfo('Request --> %s, Status --> %s' % (response.reason, response.status))
        return response
    
    def xml_components(self, xml):
        """ Find all elements in the updated xml.
        root.find requires namespaces to be a dictionary.
        Return sequence and elements to process ROS msgs.
        """
        root = ElementTree.fromstring(xml)
        ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')
        header = root.find('.//m:Header', namespaces=ns)
        nextSeq = header.attrib['nextSequence']
        #elements = root.findall('.//m:Events/*', namespaces=ns)
        
        elements = []
        #print 'self.action_list --> %s' % self.action_list
        for action in self.action_list:
            find_action = root.findall('.//m:' + action, namespaces=ns)
            if find_action: # Element list is not empty
                elements.append(find_action[0])
        return nextSeq, elements
    
    def action_client(self, cb_data):
        
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
        
        try:
            for attrib, xml_goal in zip(goal_handle.__slots__, self.xml_goal):
                co_str = 'goal.' + attrib + ' = xml_goal'
                print co_str
                co_exec = compile(co_str, '', 'exec')
                exec(co_exec)
            
            print goal
        except Exception as e:
            rospy.logerr("ROS Action %s Client failed: %s" % (name, e))
        
        # Sends the goal to the action server.
        rospy.loginfo('Sending the goal')
        client.send_goal(goal, done_cb = None, active_cb = self.action_cb)
        
        #print client.get_state()
        #print actionlib.actionlib_msgs.msg.GoalStatus.status
        #if client.get_state() == actionlib.actionlib_msgs.msg.GoalStatus.status:
        #    # Respond that goal is accepted
        #    rospy.loginfo("Changing Material Load to 'ACTIVE'")
        #    self.adapter.begin_gather()
        #    self.material_load_di.set_value('ACTIVE')
        #    self.adapter.complete_gather()
        
        # Waits for the server to finish performing the action.
        rospy.loginfo('Waiting for result')
        client.wait_for_result()
        
        # Obtain result
        result = client.get_result() # result must be a string
        
        # Prints out the result of the executing action
        rospy.loginfo('Returning the result --> %s' % result)
        print dir(result)
        
        # Set the Robot XML data item
        tokens = re.findall(r'([A-Z][a-z]*)', name)
        tokenlist = [val.lower() for val in tokens]
        data_item = tokenlist[0] + '_' + tokenlist[1]
        
        
        di_result = getattr(result, result.__slots__[0])
        
        self.adapter.begin_gather()
        co_str = "self." + data_item + "_di.set_value('" + di_result + "')"
        co_exec = compile(co_str, '', 'exec')
        exec(co_exec)
        self.adapter.complete_gather()
        
        return

    def xml_callback(self, chunk):
        rospy.loginfo('*******************In PROCESS_XML callback***************')
        self.lock.acquire()
        try:
            if self.trigger == None:
                #conn = HTTPConnection('localhost', self.config['localhost'])
                #response = self.xml_get_response(conn, "/cnc/current")
                #chunk = response.read()
                
                #print 'XML\n'
                #print chunk
                
                _, elements = self.xml_components(chunk)
                print 'ELEMENTS --> %s' % elements
                
                if elements:
                    for e in elements:
                        print e.tag, e.text
                        action_text = re.findall(r'(?<=\})\w+',e.tag)[0] 
                        if action_text in self.di_current.keys() and e.text == 'ACTIVE':
                            # Set ROS action trigger
                            self.trigger = 'ACTIVE'
                            
                            # Determine goal as specified in XML
                            conn = HTTPConnection('localhost', self.config['localhost'])
                            response = self.xml_get_response(conn, "/cnc/current")
                            chunk = response.read()
                            root = ElementTree.fromstring(chunk)
                            ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')
                            
                            goal_str = self.config[self.namespace][action_text].keys()[0]
                            goal_element = root.findall('.//m:' + goal_str, namespaces=ns)[0]
                            self.xml_goal = (goal_element.text, 32.2, 7.4) # Should be a tuple of (material type, length, diameter)
                            self.ros_action((action_text, self.action_goals[action_text],self.type_handle))
        except Exception as e:
            rospy.logerr("Bridge Publisher: Process XML callback failed: %s, releasing lock" % e)
        finally:
            self.lock.release()
        rospy.loginfo('*******************Done with PROCESS_XML callback***************')
        return

    def ros_action(self, cb_data):
        rospy.loginfo('-------------------In ROS_ACTION callback---------------')
        #self.lock.acquire()
        print 'self.trigger = %s' % self.trigger
        
        name, goal_attr, handle = cb_data
        
        try:
            if self.trigger == 'ACTIVE':
                # Execute ROS Action
                rospy.init_node('ActionClient')
                self.action_client((name, goal_attr, handle))
                self.trigger = None
        except Exception as e:
            rospy.logerr("Bridge Publisher: ROS-Publisher callback failed: %s, releasing lock" % e)        
        finally:
            pass
            #self.lock.release()
        rospy.loginfo('-------------------Done with ROS_ACTION callback---------------')
        return
    
    def action_cb(self):
        # Respond that goal is accepted
        rospy.loginfo("Changing Material Load to 'ACTIVE'")
        self.adapter.begin_gather()
        self.material_load_di.set_value('ACTIVE')
        self.adapter.complete_gather()
        time.sleep(10) # for debug only -- to be removed when generic server is running
        return

if __name__ == '__main__':
    try:
        robot_action = GenericActionClient()
        #result = material_load_client()
        #rospy.loginfo('Action Result --> %s' % result)
    except rospy.ROSInterruptException:
        rospy.loginfo('program interrupted before completion')
        