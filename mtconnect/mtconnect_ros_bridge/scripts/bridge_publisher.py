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
import thread # for thread locking
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
from long_pull import LongPull

# Import ROS Python modules
import roslib
import rospy

## @class BridgePublisher
## @brief The BridgePublisher
## parses XML data from an HTTP connection and starts a ROS publishing
## thread that publishes topic data at 10 hertz.  Topics and topic parameters are specified
## by a configuration file that must be included with the main program during execution.
## If this file is not provided, the node will terminate with an error message indicating
## the need for this file.
##
## Command line example:
##
##     bridge_publisher.py -i bridge_publisher_config.yaml
##     bridge_publisher.py -input bridge_publisher_config.yaml
##
## The class contains the following methods:
##
## setup_topic_data -- utilizes introspection to set up topic instance variables.
## init_di_dict -- creates hash table of data_item:None pairs.
## process_xml -- parses xml elements and updates the di_current and di_changed dictionaries.
## topic_publisher -- gets and sets attributes for a ROS message. Publishes ROS message.
## xml_callback -- callback function invoked by longpull, executes process_xml function and stores copy of di_changed into XML_queue.
## ros_publisher -- function that uses a Timer thread to publish ROS topic.  Extracts data from XML_queue and executes topic_publisher.
class BridgePublisher():
    ## @brief Constructor for a BridgePublisher
    def __init__(self):
        # Initialize the ROS publisher node
        rospy.init_node('bridge_publisher')
        
        # Setup MTConnect to ROS Conversion
        self.config = bridge_library.obtain_dataMap()
        self.msg_parameters = ['url', 'url_port', 'machine_tool', 'xml_namespace']
        self.url = self.config[self.msg_parameters[0]]
        self.url_port = self.config[self.msg_parameters[1]]
        self.mtool = self.config[self.msg_parameters[2]]
        self.ns = dict(m = self.config[self.msg_parameters[3]])
        
        # Check for url connectivity, dwell until system timeout
        bridge_library.check_connectivity((1,self.url,self.url_port))
        
        # Create the data lists for the topic names, types, manifests, data items, ROS publishers, and ROS messages
        self.topic_name_list = []
        self.topic_type_list = []
        self.lib_manifests = []
        self.data_items = []
        self.pub = []
        self.msg = []
        self.data_hold = None
        
        # Setup topic, data items, pub, etc. via configuration file
        self.setup_topic_data()
        
        # Hash table of persistent CNC event states, only updates when state changes
        self.di_current = self.init_di_dict()
        
        # Hash table to store changed CNC event states
        self.di_changed = None
        
        # Establish XML connection, read in current XML
        while True:
            try:
                self.conn = HTTPConnection(self.url, self.url_port)
                response = bridge_library.xml_get_response((self.url, self.url_port, None, self.conn, self.mtool + "/current"))
                body = response.read()
                break
            except socket.error as e:
                rospy.loginfo('HTTP connection error %s' % e)
                time.sleep(10)
        
        # Parse the XML and determine the current sequence and XML Event elements
        seq, elements = bridge_library.xml_components(body, self.ns, self.di_current)
        
        # Use XML to establish current data item state dictionary
        for di in self.di_current.keys():
            name = bridge_library.split_event(di)
            for e in elements:
                rospy.loginfo('Element %s --> %s' % (e.tag, e.text))
                if name == e.attrib['name']:
                    self.di_current[di] = e.text
        
        # Confirm that data items in config file are present in XML
        for di_set in self.data_items:
            if not set(di_set).issubset(set(self.di_current.keys())):
                rospy.logerr('ERROR: INCORRECT EVENTS IN TOPIC CONFIG OR XML AGENT FILE')
                sys.exit()
        
        # Start a streaming XML connection
        response = bridge_library.xml_get_response((self.url, self.url_port, None, self.conn, self.mtool + "/sample?interval=1000&count=1000&from=" + seq))
        
        # Create queue for streaming XML
        self.XML_queue = Queue()
        
        # Create a ROS publishing thread
        self.ros_publisher()
        
        # Streams data from the agent...
        lp = LongPull(response)
        lp.long_pull(self.xml_callback) # Runs until user interrupts

    ## @brief This function captures the topic name, type, and member
    ## attributes that are required for the ROS publisher.  This task
    ## is completed for each topic specified in the configuration file.
    ## 
    ## This function then performs a relative import of the topic
    ## via the getattr(import_module) function.  Data is stored in the
    ## following class attributes:
    ##    
    ##     self.data_items   --> used for ROS data structure tracking, stores list of data items for each topic.
    ##     self.pub          --> used to setup ROS topic publishers.
    ##     self.msg          --> data structure for msg class instances of the topic type.
    ##     self.topic_name_list  --> used for configuration file key reference.
    ##     self.topic_type_list  --> used for module import and config file key reference.
    def setup_topic_data(self):
        for topic_name, type_name in self.config.items():
            if topic_name not in self.msg_parameters:
                # TOPIC --> topic name, such as 'chatter'
                # TOPIC TYPE --> ROS package and message type, such as mtconnect_msgs/CncStatus
                tn = type_name.keys()[0]
                tokens = tn.split('/')
                package = tokens[0]
                topic_type_name = tokens[1]
                
                # Load package manifest if unique
                if package not in self.lib_manifests:
                    roslib.load_manifest(package)
                    self.lib_manifests.append(package)
    
                # Import module and create topic type class,
                #    i.e. append <class 'mtconnect_msgs.msg._RobotStates.RobotStates'>
                rospy.loginfo('Class Instance --> ' + package + '.msg.' + topic_type_name)
                type_handle = getattr(import_module(package + '.msg'), topic_type_name)
                
                # Append list of data items specified in the ROS topic message
                self.data_items.append(self.config[topic_name][type_name.keys()[0]].keys())
                
                # Create ROS publisher
                self.pub.append(rospy.Publisher(topic_name, type_handle))
                
                # Instance of topic type message class
                self.msg.append(type_handle())
                
                # Capture topic name and type for self.config reference
                self.topic_name_list.append(topic_name)
                self.topic_type_list.append(type_name.keys()[0]) # Only one type per topic
        
        rospy.loginfo('TOPIC NAME LIST --> %s' % self.topic_name_list)
        
        return
    
    ## @brief Initialize the data item dictionary with data item : None pairs.
    ## Dictionary used to record only changed event states.
    ## @return A dictionary of data_item:None pairs for every data item in the configuration file.
    ## Even if multiple topics are used, all of the data items are stored in a single hash table.
    def init_di_dict(self):
        return {di:None for di_list in self.data_items for di in di_list}
    
    ## @brief Function determines if an event of interest has changed and updates
    ## the current and changed data item dictionaries.  Events that will be
    ## published are specified by the user in a .yaml file.
    ## @param xml: xml data, read from response.read()
    ## @return integer representing the next sequence to be added to XPath expression for streaming
    ## @return dictionary of changed data items that includes the ROS attribute value for the ROS msg data structure.
    def process_xml(self, xml):
        nextSeq, elements = bridge_library.xml_components(xml, self.ns, self.di_current)

        # Create local data item dictionary to track event changes
        local_di = self.init_di_dict()

        # Loop through XML elements, update the di_changed and di_current
        di_list = [di_val for di_list in self.data_items for di_val in di_list]
        
        if elements:
            rospy.logdebug('***********XML -->*************\n%s' % xml)
            for e in elements:
                for d_item in di_list:
                    if e.attrib['name'] == bridge_library.split_event(d_item):
                        local_di[d_item] = e.text
                        self.di_current[d_item] = e.text # Update di_current with new state
                        break
        return nextSeq, local_di
    
    ## @brief Publishes the MTConnect message.  This function is a generic publisher.
    ## If the machine tool xml changed, the message attribute is updated with this value.
    ## Otherwise the message attribute is updated with the value stored in the di_current dictionary.
    ## @param cb_data: tuple containing the following parameters:
    ## @param topic_name: string containing the name of the ROS topic
    ## @param topic_type: string containing the name of the topic type
    ## @param pub: ROS publisher object for the topic_name and topic_type
    ## @param msg: ROS message class instance for the topic_name and topic_type
    ## @param data_items: list containing a string of data items for the specified topic
    def topic_publisher(self, cb_data):
        # Unpack function arguments
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
                    
                    # Convert from MTConnect to ROS format from di_changed
                    ros_tag = self.config[topic_name][topic_type][di_name][self.di_changed[di_name]]
                else:
                    # Convert from MTConnect to ROS format via di_current
                    ros_tag = self.config[topic_name][topic_type][di_name][self.di_current[di_name]]
                
                # Obtain the state value via operator.attrgetter object: 'door_state.CLOSED'
                name = bridge_library.split_event(di_name)
                state_object = operator.attrgetter(name + '.' + ros_tag)
                state_value = state_object(msg)
                
                # From the msg class, obtain the message attribute using data item name: attrib = msg.door_state
                attrib = getattr(msg, name)
                
                # Set the attribute to the converted ROS value: setattr(msg, 'door_state.val', 1) --> msg.door_state.val = 1
                setattr(attrib, attrib.__slots__[0], state_value)

        pub.publish(msg)
        return
    
    ## @brief Processes the xml chunk provided by the LongPull class instance and stores the result
    ## into di_changed since the tags have changed.  di_changed is then stored into a queue
    ## to be used by ros_publisher.  The queue is used to ensure capture of updated tags while
    ## ros_publisher is blocked publishing a message.
    ## @param chunk: xml data, read from response.read()
    def xml_callback(self, chunk):
        #rospy.loginfo('*******************In PROCESS_XML callback***************')
        try:
            _, self.di_changed = self.process_xml(chunk)
            if self.di_changed is not None:
                self.XML_queue.put(self.di_changed)
                rospy.logdebug('PUTTING XML INTO QUEUE %s\tNUMBER OF QUEUED OBJECTS %s' % (self.XML_queue, self.XML_queue.qsize()))
                if self.XML_queue.qsize() > 1:
                    rospy.loginfo('STORED XML INTO QUEUE, WAITING ON ROS PUBLISHER, QUEUE SIZE --> %s' % self.XML_queue.qsize())
        except Exception as e:
            rospy.logerr("Bridge Publisher: Process XML callback failed: %s, releasing lock" % e)
        finally:
            pass
        #rospy.loginfo('*******************Done with PROCESS_XML callback***************')
        return
    
    ## @brief Starts a publishing thread at 10 hertz.  Data required for the ROS publisher
    ## is obtained from the XML queue.  If data is not available, the function will not launch
    ## the ROS publisher.  If the queue is empty, data publishes the previous message.
    def ros_publisher(self):
        #rospy.loginfo('-------------------In ROS_PUBLISHER callback---------------')

        # Create timer thread
        ROSpub = Timer(0.1, self.ros_publisher)
        ROSpub.daemon = True
        ROSpub.start()
        
        # Pull from queue if XML is available, if not use stored value
        if self.XML_queue.empty() and self.data_hold is not None:
            data = self.data_hold
        elif not self.XML_queue.empty():
            data = self.XML_queue.get()
        else:
            # XML data is not available, do not publish
            data = None
        
        try:
            if data != None:
                # Publish all topics specified in the configuration file
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
            if data != self.data_hold:
                # Update the persistent variable and close the queue
                self.data_hold = data
                self.XML_queue.task_done()
            else:
                pass

        #rospy.loginfo('-------------------Done with ROS_PUBLISHER callback---------------')
        return

if __name__ == '__main__':
    try:
        rospy.loginfo('Launching Bridge Publisher -- Publishing Machine Tool Messages')
        mtc_parse = BridgePublisher()
    except rospy.ROSInterruptException:
        sys.exit(0)
