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

## @package bridge_subscriber.py
## This module launches a ROS node that will subscribe to ROS topics specified in the
## configuration file and then ports the data to the machine tool adapter.  Topics and
## topic parameters are specified by a configuration file that must be included with
## the main program during execution. If this file is not provided, the node will terminate
## with an error message indicating the need for this file.
##
## Command line example:
##
##     bridge_subscriber.py -i bridge_subscriber_config.yaml
##     bridge_subscriber.py -input bridge_subscriber_config.yaml

# Import standard Python modules
import sys
import os
import optparse
import yaml
import operator
import thread
import re
import time
from importlib import import_module

# Import custom ROS-MTConnect function library
import bridge_library

# Import custom Python modules for MTConnect Adapter interface
path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from mtconnect_adapter import Adapter

# Import ROS Python modules
import roslib
import rospy

## @class BridgeSubscriber
## @brief The BridgeSubscriber
## will subscribe to ROS topics specified in the configuration file and then ports the data
## to the machine tool adapter.  The class contains the following methods:
##
## setup_topic_data -- utilizes introspection to set up class instance variables
## topic_callback -- callback function that captures the attribute values for a ROS topic
## data_item_conversion -- sets MTConnect adapter value via ROS message CONSTANT value
## topic_listener -- ROS subscriber function that launches subscribers
class BridgeSubscriber():
    ## @brief Constructor for a BridgeSubscriber
    def __init__(self):
        # Initialize the ROS Bridge subscriber node
        rospy.init_node('bridge_subscriber')
        
        # Read configuration file and extract topics and types
        self.config = bridge_library.obtain_dataMap()
        self.msg_parameters = ['url', 'adapter_port']
        self.url = self.config[self.msg_parameters[0]]
        self.adapter_port = self.config[self.msg_parameters[1]]
        
        # Setup MTConnect Adapter for robot status data items
        self.adapter = Adapter((self.url, self.adapter_port))
        self.di_dict = {} # XML data item dictionary to store MTConnect Adapter Events
        
        # Setup ROS topics as specified in .yaml file
        self.topic_name_list = [] # List of topic names
        self.subscribed_list = [] # List of subscribed topics
        self.lib_manifests = [] # Stores loaded ROS library manifests

        # Topic type and MTConnect message members from the module import
        self.topic_type_list = {}
        self.member_types = {}
        self.member_names = {}
        
        # The ROS message text retainer, used to map ROS values to MTConnect XML tag.text 
        self.msg_text = {}
        
        # Create the data sets for the topic types, member names, and member types
        self.setup_topic_data()
        
        # Start the adapter
        rospy.loginfo('Start the Robot Link adapter')
        self.adapter.start()
        
        # Create class lock
        self.lock = thread.allocate_lock()
        
        # Loop through requested topics and spawn ROS subscribers when topics are available
        topic_list = self.topic_name_list
        dwell = 10
        while topic_list:
            published_topics = dict(rospy.get_published_topics())
            for topic_name in topic_list:
                if topic_name in published_topics.keys():
                    # Create ROS Subscriber
                    idx = topic_list.index(topic_name)
                    del topic_list[idx]
                    self.topic_listener((topic_name, self.topic_type_list[topic_name], self.member_names[topic_name], self.msg_text[topic_name]))
                else:
                    rospy.loginfo('ROS Topic %s not available, will try to subscribe in %d seconds' % (topic_name, dwell))
                    time.sleep(dwell)
    
    ## @brief This function captures the topic name, type, and member attributes that are required for 
    ## the ROS subscriber.  This task is completed for each topic specified in the configuration file.
    ## 
    ## This function then performs a relative import of the topic via the getattr(import_module) function.
    ## Data is stored in the following class instance attributes:
    ## 
    ##     self.topic_type_list --> used for module import and msg parameters
    ##     self.member_types    --> member type, not used in msg parameters, future use
    ##     self.member_names    --> used for ROS subscriber msg parameters 
    def setup_topic_data(self):
        for topic, topic_type in self.config.items():
            if topic not in self.msg_parameters:
                self.topic_name_list.append(topic)
                
                # Only one type per topic
                type_key = topic_type.keys()[0]
                self.subscribed_list.append(type_key)
                
                # One set of data items per topic type
                self.data_items = [data_item for data_item in topic_type[type_key].keys()]
        
                # Add data items to the MTConnect Adapter - must be unique data items
                bridge_library.add_event((self.adapter, self.data_items, self.di_dict, False))
                
                # Extract package namespace and topic type name
                tokens = topic_type.keys()[0].split('/')
                namespace = tokens[0]
                type_name = tokens[1]
                
                # Load package manifest if unique
                if namespace not in self.lib_manifests:
                    roslib.load_manifest(namespace)
                    self.lib_manifests.append(namespace)
                
                # Import module and create topic type class,
                #    i.e. append <class 'mtconnect_msgs.msg._RobotStates.RobotStates'>
                rospy.loginfo('Class Instance --> ' + namespace + '.msg.' + type_name)
                type_handle = getattr(import_module(namespace + '.msg'), type_name)

                self.topic_type_list[topic] = type_handle
                self.msg_text[topic] = type_handle._full_text
                self.member_types[topic] = type_handle._slot_types
                self.member_names[topic] = type_handle.__slots__
        return
    
    ## @brief Callback function that captures the attribute values for a ROS topic.
    ## The topic data is stored in a list of tuples, where each tuple is a
    ## (attrib_name, attrib_value) pair.  To access the integer value of the attribute
    ## value, use attrib_value.val.
    ## 
    ## All data conversions between ROS and MTConnect are stored in the ROS
    ## subscriber .yaml file. A separate function handles the ROS to MTConnect
    ## conversions for generic robot messages.
    ## @param data: callback ROS message data from the ROS subscriber
    ## @param cb_data: tuple containing the following parameters:
    ## @param topic_name: string defining the name of the ROS topic
    ## @param type_handle: class instance of the ROS topic message i.e. <class 'mtconnect_msgs.msg._RobotStates.RobotStates'>
    ## @param member_set: list of strings defining the attribute members for the message class
    ## @param msg_text: string of the entire message text (may be eliminated in future revisions)
    def topic_callback(self, data, cb_data):
        self.lock.acquire()
        try:
            (topic_name, type_handle, member_set, msg_text) = cb_data
            type_name = type_handle._type
            
            # Repackage members into a dictionary
            dout = {val:val for val in member_set}

            # Iterate through data items and capture message data
            #rospy.loginfo('Message on %s for %s' % (topic_name, rospy.get_name()))
            msg_data = []
            msg_constants = {}
            for dataitem in member_set:
                # Capture attribute for namespace/Topic Type --> industrial_msgs/TriState.avail
                attrib_handle = getattr(type_handle(), dataitem)
                
                if 'header' not in dataitem:
                    # Extract string representing message attribute --> 'val'
                    val_key = attrib_handle.__slots__[0]
                    
                    # Capture message data.attrib --> <class message data>.dataitem: TriState.avail
                    token = getattr(data, dataitem)
                    
                    # Capture the integer value for the attribute: avail.val
                    item_value = getattr(token, val_key)
                    
                    # Store published message data for the topic
                    msg_data.append((dataitem, item_value))
                    
                    # Create a list of strings containing message CONSTANTS
                    msg_constants[dataitem] = []
                    for attrib in dir(attrib_handle):
                        if attrib.isupper():
                            if getattr(attrib_handle, attrib) == item_value:
                                msg_constants[dataitem].append(attrib)
                    #rospy.loginfo('MESSAGE CONSTANTS --> %s\tVAL --> %s' % (msg_constants, item_value))
            
            # Execute the ROS to MTConnet conversion function
            self.data_item_conversion(topic_name, type_name, msg_data, msg_text, msg_constants)
            
        except Exception as e:
            rospy.logerr('Topic callback failed: %s, releasing lock' % e)
        finally:
            self.lock.release()
        return
    
    ## @brief A conversion function that will convert the ROS message value to the 
    ## machine tool value.  The conversion dictionary is provided in the configuration file.
    ## @param topic_name: string defining the name of the ROS topic
    ## @param type_name: string defining the name of the ROS topic type
    ## @param msg_data: list of strings defining message attributes
    ## @param msg_text: string of the entire message text (may be eliminated in future revisions)
    ## @param constants: list of ROS topic message type CONSTANTS stored as strings
    def data_item_conversion(self, topic_name, type_name, msg_data, msg_text, constants):
        # Set the Robot XML data item via the MTConnect Adapter
        for member_data in msg_data:
            
            # Iterate through list of ROS message CONSTANTS and set the MTConnect Adapter value
            for const_val in constants[member_data[0]]:
                if const_val in self.config[topic_name][type_name][member_data[0]].keys():
                    # If ROS to MTConnect mapping dictionary contains the constant, set the adapter value 
                    adapter_val = self.config[topic_name][type_name][member_data[0]][const_val]
                    break
                else:
                    adapter_val = None
            
            if adapter_val is None:
                rospy.logerr('ROS to MTConnect Mapping failed')

            # Set the Robot XML data item
            bridge_library.action_cb((self.adapter, self.di_dict, member_data[0], adapter_val))
        return   
    
    ## @brief Main ROS subscriber function.  A new thread is created for each callback.
    ## @param data: tuple containing the following parameters:
    ## @param topic_name: string defining the name of the ROS topic
    ## @param type_handle: class instance of the ROS topic message i.e. <class 'mtconnect_msgs.msg._RobotStates.RobotStates'>
    ## @param member_set: list of strings defining the message attributes
    ## @param msg_text: string of the entire message text (may be eliminated in future revisions) 
    def topic_listener(self, data):
        # Unpack arguments
        topic_name, type_handle, member_set, msg_text = data
        
        # Launch the ROS subscriber
        rospy.Subscriber(topic_name, type_handle, self.topic_callback, data)
        return

#--------------------------------------------------
# Main program
if __name__ == '__main__':
    try:
        rospy.loginfo('Executing robot topic subscriber')
        robot_topic = BridgeSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
