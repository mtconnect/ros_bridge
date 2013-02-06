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
import operator
import thread # for thread locking
import re
import time
from Queue import Queue
from threading import Thread
from importlib import import_module
from httplib import HTTPConnection
from xml.etree import ElementTree

# Import custom Python module to read config file
import read_config_file

# Import custom Python modules for MTConnect Adapter interface
path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')
from long_pull import LongPull

# Import ROS Python modules
import roslib
import rospy

class BridgePublisher():
    def __init__(self):
        # Initialize the ROS publisher node
        rospy.init_node('bridge_publisher')
        
        # Setup MTConnect to ROS Conversion
        self.config = read_config_file.obtain_dataMap()
        self.msg_parameters = ['url', 'url_port', 'machine_tool', 'xml_namespace']
        self.url = self.config[self.msg_parameters[0]]
        self.url_port = self.config[self.msg_parameters[1]]
        self.mtool = self.config[self.msg_parameters[2]]
        self.xml_ns = self.config[self.msg_parameters[3]]
        
        # Create the data lists for the topic names, types, manifests, data items, ROS publishers, and ROS messages
        self.topic_name_list = []
        self.topic_type_list = []
        self.lib_manifests = []
        self.data_items = []
        self.pub = []
        self.msg = []
        
        # Setup topic, data items, pub, etc. via configuration file
        self.setup_topic_data()
        
        # Hash table of persistent CNC event states, only updates when state changes
        self.di_current = {}
        
        # Hash table to store changed CNC event states
        self.di_changed = None
        
        # Establish XML connection, read in current XML
        conn = HTTPConnection(self.url, self.url_port)
        response = self.xml_http_connection(conn, self.mtool + "/current")
        body = response.read()
        
        # Parse the XML and determine the current sequence and XML Event elements
        seq, elements = self.xml_components(body)
        
        # Use XML to establish current data item state dictionary
        self.di_current = {e.attrib['name']:e.text for e in elements if 'name' in e.attrib.keys()}
        
        # Confirm that data items in config file are present in XML
        for di_set in self.data_items:
            if not set(di_set).issubset(set(self.di_current.keys())):
                rospy.logerr('ERROR: INCORRECT EVENTS IN TOPIC CONFIG OR XML AGENT FILE')
                sys.exit()
        
        # Start a streaming XML connection
        response = self.xml_http_connection(conn, self.mtool + "/sample?interval=1000&count=1000&from=" + seq)
        
        # Create class lock
        self.lock = thread.allocate_lock()

        # Create publishing thread
        #rospy.Timer(rospy.Duration(0.1), self.ros_publisher)
        self.XML_queue = Queue()
        ROSpub = Thread(target = self.ros_publisher, args=())
        ROSpub.setDaemon(True)
        ROSpub.start()

        # Streams data from the agent...
        lp = LongPull(response)
        lp.long_pull(self.xml_callback) # Runs until user interrupts
    
    def setup_topic_data(self):
        """This function captures the topic name, type, and member
        attributes that are required for the ROS publisher.  This task
        is completed for each topic specified in the configuration file.
        
        This function then performs a relative import of the topic
        via the getattr(import_module) function.  Data is stored in the
        following class attributes:
        
            self.data_items   --> used for ROS data structure tracking, stores list of data items for each topic
            self.pub          --> used to setup ROS topic publishers
            self.msg          --> data structure for msg class instances of the topic type
            self.topic_name_list  --> used for configuration file key reference
            self.topic_type_list  --> used for module import and config file key reference
        """
        
        for topic_name, type_name in self.config.items():
            if topic_name not in self.msg_parameters:
                # TOPIC --> topic name, such as 'chatter'
                # TOPIC TYPE --> message namespace and name, such as mtconnect_msgs/CncStatus
                tn = type_name.keys()[0]
                tokens = tn.split('/')
                namespace = tokens[0]
                topic_type_name = tokens[1]
                
                # Load package manifest if unique
                if tokens[0] not in self.lib_manifests:
                    roslib.load_manifest(tokens[0])
                    self.lib_manifests.append(tokens[0])
    
                # Import module and create topic type class,
                #    i.e. append <class 'mtconnect_msgs.msg._RobotStates.RobotStates'>
                rospy.loginfo('Class Instance --> ' + namespace + '.msg.' + topic_type_name)
                type_handle = getattr(import_module(namespace + '.msg'), topic_type_name)
                
                # Append list of data items specified in the ROS topic message
                self.data_items.append([val for val in type_handle.__slots__ if val != 'header'])
                
                # Create ROS publisher
                self.pub.append(rospy.Publisher(topic_name, type_handle))
                
                # Instance of topic type message class
                self.msg.append(type_handle())
                
                # Capture topic name and type for self.config reference
                self.topic_name_list.append(topic_name)
                self.topic_type_list.append(type_name.keys()[0]) # Only one type per topic
        
        rospy.loginfo('TOPIC NAME LIST --> %s' % self.topic_name_list)
        
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

    def xml_components(self, xml):
        """ Find all elements in the updated xml. root.find requires namespaces to be a dictionary.
        Return sequence and elements to process ROS msgs.
        """
        root = ElementTree.fromstring(xml)
        ns = dict(m = self.xml_ns)
        header = root.find('.//m:Header', namespaces=ns)
        nextSeq = header.attrib['nextSequence']
        elements = root.findall('.//m:Events/*', namespaces=ns)        
        return nextSeq, elements
    
    def init_di_dict(self):
        """ Initialize the data item dictionary with data item : None
        pairs.  Dictionary used to record only changed event states.
        """
        return {val:None for item in self.data_items for val in item}
    
    def process_xml(self, xml):
        """Function determines if an event of interest has changed and updates
        the current and changed data item dictionaries.  Events that will be
        published are specified by the user in a .yaml file.  The function 
        returns the updated changed data item dictionary that includes the
        ROS attribute value for the ROS msg data structure.
        """
        nextSeq, elements = self.xml_components(xml)
        
        # Create local data item dictionary to track event changes
        local_di = self.init_di_dict()

        # Loop through XML elements, update the di_changed and di_current
        for e in elements:
            rospy.loginfo('Changing ROS message --> %s --> %s' % (e.tag, e.text))
            
            di_list = [val for di_vals in self.data_items for val in di_vals]

            if 'name' in e.attrib.keys():
                if e.attrib['name'] in di_list:
                    # Change local_di from None to current MTConnect event status
                    di = e.attrib['name']
                    local_di[di] = e.text
                    self.di_current[di] = e.text # Update di_current with new state
        return nextSeq, local_di

    def topic_publisher(self, cb_data):
        """ Publishes the MTConnect message.  This function is a generic publisher.
        Attribute fetches are obtained via the attrgetter function in the operator
        module.  In order to assign a value to a relative attribute, a code object
        is created and then executed.  
        """
        topic_name, topic_type, pub, msg, data_items = cb_data
        
        msg.header.stamp = rospy.Time.now()
        
        for di_name, value in self.di_changed.items():
            # Convert tag text from MTConnect to ROS and set attribute value
            if di_name in data_items:
                if value is not None:
                    # Check for conversion key error
                    conv_keys = self.config[topic_name][topic_type][di_name].keys()
                    if value not in conv_keys:
                        rospy.logerr("CONVERSION ERROR IN TOPIC CONFIG FILE: XML is '%s' --> CONFIG is %s" % (value, conv_keys))
                        os._exit(0)
                    
                    # Used changed value in di_changed
                    rospy.loginfo('%s changed, set from di_changed' % di_name) # DEBUG
                    
                    # Convert from MTConnect to ROS format from di_changed
                    ros_tag = self.config[topic_name][topic_type][di_name][self.di_changed[di_name]]
                else:
                    # Use stored value in di_current
                    # Convert from MTConnect to ROS format via di_current
                    ros_tag = self.config[topic_name][topic_type][di_name][self.di_current[di_name]]
                
                # Obtain the state value via operator.attrgetter object: 'door_state.CLOSED'
                state_object = operator.attrgetter(di_name + '.' + ros_tag)
                state_value = state_object(msg)
                
                # From the msg class, obtain the message attribute using data item name: attrib = msg.door_state
                attrib = getattr(msg, di_name)
                
                # Set the attribute to the converted ROS value: setattr(msg, 'door_state.val', 1) --> msg.door_state.val = 1
                setattr(attrib, attrib.__slots__[0], state_value)

        pub.publish(msg)
        return

    def xml_callback(self, chunk):
        #rospy.loginfo('*******************In PROCESS_XML callback***************')
        #self.lock.acquire()
        try:
            _, self.di_changed = self.process_xml(chunk)
            self.XML_queue.put(self.di_changed)
            rospy.loginfo('PUTTING XML INTO QUEUE %s\tNUMBER OF QUEUED OBJECTS %s' % (self.XML_queue, self.XML_queue.qsize()))
        except Exception as e:
            rospy.logerr("Bridge Publisher: Process XML callback failed: %s, releasing lock" % e)
        finally:
            #self.lock.release()
            pass
        #rospy.loginfo('*******************Done with PROCESS_XML callback***************')
        return
        
    #def ros_publisher(self, event):
    def ros_publisher(self):
        #rospy.loginfo('-------------------In ROS_PUBLISHER callback---------------')
        #self.lock.acquire()
        while True:
            data = self.XML_queue.get()
            
            try:
                #if self.di_changed != None:
                if data != None:
                    publishers = []
                    for topic_name, topic_type, pub, message, di in zip(self.topic_name_list, self.topic_type_list,
                                                                    self.pub, self.msg, self.data_items):
                        # Publish topic
                        rospy.init_node('bridge_publisher')
                        publishers.append(self.topic_publisher((topic_name, topic_type, pub, message, di)))
                    
                    # Reset current data items to None
                    self.di_changed = self.init_di_dict()
            except Exception as e:
                rospy.logerr("Bridge Publisher: ROS-Publisher callback failed: %s, releasing lock" % e)        
            finally:
                #self.lock.release()
                self.XML_queue.task_done()
        #rospy.loginfo('-------------------Done with ROS_PUBLISHER callback---------------')
        return

if __name__ == '__main__':
    try:
        rospy.loginfo('Launching Bridge Publisher -- Publishing Machine Tool Messages')
        mtc_parse = BridgePublisher()
    except rospy.ROSInterruptException:
        sys.exit(0)
