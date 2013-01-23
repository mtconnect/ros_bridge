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
import thread
import re
import time
from importlib import import_module

# Import custom Python module to read config file
import read_config_file

# Import custom Python modules for MTConnect Adapter interface
path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from mtconnect_adapter import Adapter

# Import ROS Python modules
import roslib
import rospy

class BridgeSubscriber():
    def __init__(self):
        # Read configuration file and extract topics and types
        self.config = read_config_file.obtain_dataMap()
        #rospy.loginfo(self.config)
        
        # Setup MTConnect Adapter for robot status data items
        self.adapter = Adapter(('0.0.0.0', self.config['Port']))
        
        # Setup ROS topics as specified in .yaml file
        self.topic_name_list = [] # List of topic names
        self.subscribed_list = [] # List of subscribed topics
        self.lib_manifests = [] # Stores loaded ROS library manifests

        for topic, topic_type in self.config.items():
            if topic != 'Port':
                self.topic_name_list.append(topic)
                
                # Only one type per topic
                type_key = topic_type.keys()[0]
                self.subscribed_list.append(type_key)
                
                # One set of data items per topic type
                self.data_items = [data_item for data_item in topic_type[type_key].keys()]
        
                # Add data items to the MTConnect Adapter - must be unique data items
                for data_item in self.data_items:
                    co_str = "self." + data_item + "_di = Event('" + data_item + "')"
                    co_exec = compile(co_str, '', 'exec')
                    exec(co_exec)
        
                    co_str = 'self.adapter.add_data_item(self.' + data_item + '_di)'
                    co_exec = compile(co_str, '', 'exec')
                    exec(co_exec)

        # Start the adapter
        rospy.loginfo('Start the Robot Link adapter')
        self.adapter.start()
        
        # Topic type and MTConnect message members from the module import
        self.topic_type_list = []
        self.member_types = []
        self.member_names = []
        
        # The ROS message text retainer, used to map ROS values to MTConnect XML tag.text 
        self.msg_text = []
        
        # Create the data sets for the topic types, member names, and member types
        self.setup_topic_data()
        
        # Create class lock
        self.lock = thread.allocate_lock()
        
        # Create ROS subscribers
        self.topic_listener()

    def setup_topic_data(self):
        """This function verifies topic availability and captures the
        topic name, type, and member attributes that are required for 
        the ROS subscriber.  This task is completed for each topic
        specified in the configuration file.
        
        This function then performs a relative import of the topic
        via the getattr(import_module) function.  Date is stored in the
        following class attributes:
        
            self.topic_type_list   --> used for module import and msg parameters
            self.member_types --> member type, not used in msg parameters, future use
            self.member_names --> used for ROS subscriber msg parameters
        
        If the topic is not being published, then the user has the option 
        to quit or proceed.  Note that the code does not try to re-subscribe to
        the topic if it is published after this code executes.
        """
        rospy.loginfo('TOPIC NAME LIST --> %s' % self.topic_name_list)
        
        published_topics = dict(rospy.get_published_topics())
        rospy.loginfo('PUBLISHED TOPICS: %s' % published_topics)
        
        for topic_name, type_name in zip(self.topic_name_list, self.subscribed_list):
            if topic_name not in published_topics.keys():
                rospy.loginfo('WARNING: %s is not published and will not be included in the subscription' % topic_name)
                resume = False
                while resume == False:
                    ans = raw_input('Continue (C) or Quit (Q) --> ')
                    if ans.lower() == 'q':
                        sys.exit(0)
                    elif ans.lower() == 'c':
                        resume = True
                        pass
            else:
                # Extract package namespace and topic type name
                tokens = type_name.split('/')
                namespace = tokens[0]
                type_name = tokens[1]
                
                # Load package manifest if unique
                if tokens[0] not in self.lib_manifests:
                    roslib.load_manifest(tokens[0])
                    self.lib_manifests.append(tokens[0])

                # Import module and create topic type class,
                #    i.e. append <class 'mtconnect_msgs.msg._RobotStates.RobotStates'>
                rospy.loginfo('Class Instance --> ' + namespace + '.msg.' + type_name)
                type_handle = getattr(import_module(namespace + '.msg'), type_name)

                self.topic_type_list.append(type_handle)
                self.msg_text.append(type_handle._full_text)
                self.member_types.append(type_handle._slot_types)
                self.member_names.append(type_handle.__slots__)
        return
    
    def topic_callback(self, data, cb_data):
        """
        Callback function that captures the attribute values for a ROS topic.
        The topic data is stored in a list of tuples, where each tuple is a
        (attrib_name, attrib_value) pair.  To access the integer value of the attribute
        value, use attrib_value.val.
        
        All data conversions between ROS and MTConnect are stored in the ROS
        subscriber .yaml file. A separate function handles the ROS to MTConnect
        conversions for generic robot messages. 
        """
        self.lock.acquire()
        try:
            (topic_name, type_name, member_set, msg_text) = cb_data
            
            # Repackage members into a dictionary
            dout = {val:val for val in member_set}

            # Create output string
            #rospy.loginfo('Message on %s for %s' % (topic_name, rospy.get_name()))
            msg_data = []
            for dataitem in member_set:
                if 'header' not in dataitem:
                    # Acquire published data for the topic
                    item_value = operator.attrgetter(dout[dataitem])(data)
                    #rospy.loginfo('%s: --> %s' % (dataitem, item_value))
                    msg_data.append((dataitem, item_value))
            
            # Execute the ROS to MTConnet conversion function
            self.data_item_conversion(topic_name, type_name, msg_data, msg_text)
            
        except Exception as e:
            rospy.logerr('Topic callback failed: %s, releasing lock' % e)
        finally:
            self.lock.release()
        return

    def data_item_conversion(self, topic_name, type_name, msg_data, msg_text):
        # Set the Robot XML data item via the MTConnect Adapter
        for member_data in msg_data:
            
            # Create list of ROS message CONSTANTS
            constants = self.msg_const_pull(member_data[1].val, msg_text)
            
            for const_val in constants:
                if const_val in self.config[topic_name][type_name][member_data[0]].keys():
                    # If ROS to MTConnect mapping dictionary contains the constant, set the adapter value 
                    adapter_val = self.config[topic_name][type_name][member_data[0]][const_val]
                    break
                else:
                    adapter_val = None
            
            if adapter_val is None:
                rospy.logerr('ROS to MTConnect Mapping failed')

            # Set the Robot XML data item
            self.adapter.begin_gather()
            co_str = "self." + member_data[0] + "_di.set_value('" + adapter_val + "')"
            co_exec = compile(co_str, '', 'exec')
            exec(co_exec)
            self.adapter.complete_gather()
        return
    
    def msg_const_pull(self, val, msg_text):
        # Create regex to extract the message CONSTANTS based on ROS message value
        #     i.e. matches 1..n of CONSTANT=val and creates a list of constant strings
        re_str = '([A-Z]+)=' + str(val)
        regex = re.compile(re_str)
        msg_constants = regex.findall(msg_text)
        return msg_constants

    def topic_listener(self):
        """Main ROS subscriber function.  A new thread is created for each callback.
        self.topic_name_list and self.data_items determined from the configuration file.
        The remaining callback data derived from the setup_topic_data function.
        """
        subscribers = []
        for topic_name, type_handle, member_set, msg_text in zip(self.topic_name_list, 
                                                    self.topic_type_list, self.member_names, self.msg_text):
            rospy.init_node('bridge_subscriber')
            subscribers.append(rospy.Subscriber(topic_name, type_handle, self.topic_callback,
                                                (topic_name, type_handle._type, member_set, msg_text)))
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
