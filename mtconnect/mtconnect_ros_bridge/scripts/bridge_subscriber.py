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

import sys
import os

path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')

import operator
import thread
import re
import time

import roslib; roslib.load_manifest('mtconnect_ros_bridge')
import rospy

import read_config_file
from importlib import import_module
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from mtconnect_adapter import Adapter

class RobotTopicSubscriber():
    def __init__(self):
        # Setup MTConnect Adapter for robot status data items
        self.adapter_link = Adapter(('0.0.0.0', 7878))
        
        # Read configuration file and extract topics and types
        self.dataMap = read_config_file.obtain_dataMap()
        #rospy.loginfo(self.dataMap)
        
        # Add data items specified in .yaml file
        #     PyYaml does not provide an ordered dictionary, so find key for robot initialization
        #     Requires that all topics include '/' in the yaml topic key
        self.topic_name_list = []
        self.mtc_dataitems = []
        for key, val in self.dataMap.items():
            if '/' not in key:
                self.adapter_key = key
                self.robot_init_flag = 0 # Only need to set the init modes once
            else:
                self.topic_name_list.append(key)
                self.mtc_dataitems.append(val)
        
        for event in self.dataMap[self.adapter_key]:
            co_str = "self." + event + "_di = Event('" + event + "')"
            co_exec = compile(co_str, '', 'exec')
            exec(co_exec)
                
            co_str = 'self.adapter_link.add_data_item(self.' + event + '_di)'
            co_exec = compile(co_str, '', 'exec')
            exec(co_exec)

        # Start the adapters
        rospy.loginfo('Start the Robot Link adapter')
        self.adapter_link.start()
        # Provide a dwell to make connection
        time.sleep(10) # Future -- Eliminate this, use adapter feedback to determine if a connection is made
        
        # Topic type and MTConnect message members from the module import
        self.topic_type = []
        self.member_types = []
        self.member_names = []
        
        # The ROS message text retainer, used to map ROS values to MTConnect XML tag.text 
        self.msg_text = None
        
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
        
            self.topic_type   --> used for module import and msg parameters
            self.member_types --> member type, not used in msg parameters, future use
            self.member_names --> used for ROS subscriber msg parameters
        
        If the topic is not being published, then the user has the option 
        to quit or proceed.  Note that the code does not try to re-subscribe to
        the topic if it is published after this code executes.
        """
        rospy.loginfo('TOPIC NAME LIST --> %s' % self.topic_name_list)
        for val in self.topic_name_list:
            published_topics = dict(rospy.get_published_topics())
            rospy.loginfo('PUBLISHED TOPICS: %s' % published_topics)
            if val in published_topics.keys():
                topic_type_string = published_topics[val]
                #rospy.loginfo('topic_type_string --> %s' % topic_type_string) 
            else:
                rospy.loginfo('WARNING: %s is not published and will not be included in the subscription' % val)
                topic_type_string = None
                resume = False
                while resume == False:
                    ans = raw_input('Continue (C) or Quit (Q) --> ')
                    if ans.lower() == 'q':
                        sys.exit(0)
                    elif ans.lower() == 'c':
                        resume = True
                        pass

            if topic_type_string:
                type_string = topic_type_string.split('/')[1]
            
                namespace_string, mb_types, mb_names = self.get_topic_variables(topic_type_string)
                self.member_types.append(mb_types)
                self.member_names.append(mb_names)
            
                # Import module, module name stored in relative terms
                self.topic_type.append(getattr(import_module(namespace_string), type_string)) # Python >= 2.7
            else:
                pass
        return
    
    def get_topic_variables(self, topic_type_string):
        """Given a topic type as an argument, this function
        extracts the message module types and attributes
        via introspection.
        """
        # Assume topic type is in  "namespace/topic" form such as mtconnect_msgs/RobotStatus
        tokens = topic_type_string.split('/')
        namespace_string = tokens[0]
        type_string = tokens[1]
    
        # Create rostopic message module such as mtconnect_msgs.msg
        module_handle = __import__(namespace_string + '.msg')
        #print "Module '%s' handle contents: %s" % (module_handle.__name__, dir(module_handle))
        rospy.loginfo('Importing --> ' + namespace_string + '.msg')
        
        # Capture class instance of message with the given type_string,
        #         <class 'mtconnect_msgs.msg._RobotStatus.RobotStatus'>
        type_handle = module_handle.msg.__dict__[type_string]
        
        # ---------------- DEBUG ----------------
        rospy.loginfo('Type Handle Slot Types --> %s' % type_handle._slot_types)
        rospy.loginfo('Type Handle Slots --> %s' % type_handle.__slots__)
        #rospy.loginfo('Type Handle Full Text --> %s' % type_handle._full_text)
        # ---------------- DEBUG ----------------
        
        self.msg_text = type_handle._full_text
        # Create list of module attributes and return tuple of message parameters
        # (mtconnect_msgs.msg, ['std_msgs/Header', 'industrial_msgs/RobotMode'], ['header', 'mode'])
        return (namespace_string + '.msg', type_handle._slot_types, type_handle.__slots__)
    
    def topic_callback(self, data, cb_data):
        """Callback function that captures the attribute values for a ROS topic.
        The topic data is stored in a list of tuples, where each tuple is a
        (attrib_name, attrib_value) pair.  To access the integer value of the attribute
        value, use attrib_value.val.
        
        Upon startup, the robot state must be initialized via the MTConnet Adapter which
        is accomplished by the robot_adapter_init function which converts the data from 
        ROS to MTConnet and sets the appropriate adapter value.
        
        Since this is a generic call back function, future work will incorporate methods
        to handle topic message data not linked to robot state initialization.  All data
        conversions between ROS and MTConnect are stored in the ROS subscriber .yaml file.
        A separate function will be created to handle the ROS to MTConnect conversions for
        generic robot messages. 
        
        This function is a work in progress.
        """
        self.lock.acquire()
        try:
            (topic_name, type_name, member_set, mtc_data) = cb_data
        
            # Repackage members into a dictionary
            dout = {val:val for val in member_set}
        
            # Create output string
            #rospy.loginfo('Message on %s for %s' % (topic_name, rospy.get_name()))
            msg_data = []
            for member_attrib in member_set:
                if 'header' not in member_attrib:
                    # Acquire published data for the topic
                    attr_value = operator.attrgetter(dout[member_attrib])(data)
                    #rospy.loginfo('%s: --> %s' % (member_attrib, attr_value))
                    msg_data.append((member_attrib, attr_value))
            
            # Execute the ROS to MTConnet conversion function
            if self.adapter_key == type_name and self.robot_init_flag == 0:
                # Set robot initial state conversion function
                self.robot_adapter_init(msg_data)
            else:
                # Future expansion for generic message conversions
                pass
            
        except Exception as e:
            rospy.logerr('Topic callback failed: %s, releasing lock' % e)
        finally:
            self.lock.release()
        return

    def robot_adapter_init(self, msg_data):
        # ---------------- DEBUG ----------------
        #rospy.loginfo('msg_data %s' % msg_data)
        #rospy.loginfo('self.adapter_key --> %s' % self.adapter_key)
        #rospy.loginfo('type_name --> %s' % type_name)
        #rospy.loginfo('msg text length --> %d' % len(self.msg_text))
        # ---------------- DEBUG ----------------
        
        # Set values for the robot initialization adapter
        for member_data in msg_data:
            # Create list of message CONSTANTS
            msg_constants = self.msg_const_pull(member_data[1].val)
            
            # ---------------- DEBUG ----------------
            #rospy.loginfo('Member Data --> %s, %s' % (member_data[0], member_data[1].val))
            #rospy.loginfo('CONSTANTS --> %s' % msg_constants)
            #rospy.loginfo('dataMap keys --> %s' % self.dataMap[self.adapter_key][member_data[0]].keys())
            # ---------------- DEBUG ----------------
            
            adapter_val = None
            
            for const_val in msg_constants:
                if const_val in self.dataMap[self.adapter_key][member_data[0]].keys():
                    # If ROS to MTConnect mapping dictionary contains the constant, set the adapter value 
                    adapter_val = self.dataMap[self.adapter_key][member_data[0]][const_val]
                
                    # ---------------- DEBUG ----------------
                    #rospy.loginfo('adapter_val --> %s' % adapter_val)
                    # ---------------- DEBUG ----------------
                
                    break
            
            if adapter_val is None:
                rospy.logerr('ROS to MTConnect Mapping failed')
                return

            # Set Robot XML
            self.adapter_link.begin_gather()
            co_str = "self." + member_data[0] + "_di.set_value('" + adapter_val + "')"
            rospy.loginfo(co_str)
            co_exec = compile(co_str, '', 'exec')
            exec(co_exec)
            self.adapter_link.complete_gather()
            
        self.robot_init_flag = 1
        
        return
    
    def msg_const_pull(self, val):
        # Create regex to extract the message CONSTANTS based on ROS message value
        #     i.e. matches 1..n of CONSTANT=val and creates a list of constant strings
        re_str = '([A-Z]+)=' + str(val)
        regex = re.compile(re_str)
        msg_constants = regex.findall(self.msg_text)
        return msg_constants

    def topic_listener(self):
        """Main ROS subscriber function.  A new thread is created for each callback.
        self.topic_name_list and self.mtc_dataitems determined from the configuration file.
        The remaining callback data derived from the setup_topic_data function.
        """
        subscribers = []
        for each_topic, each_package, member_set, mtc_values in zip(self.topic_name_list, 
                                                                   self.topic_type, self.member_names, self.mtc_dataitems):
            rospy.init_node('bridge_subscriber')
            subscribers.append(rospy.Subscriber(each_topic, each_package, self.topic_callback,
                                                (each_topic, each_package.__name__, member_set, mtc_values)))
            # ---------------- DEBUG ----------------
            #rospy.loginfo('TOPIC --> %s' % each_topic)
            #rospy.loginfo('TYPE --> %s' % each_package.__name__)
            #rospy.loginfo('MEMBERS --> %s' % member_set)
            #rospy.loginfo('MTC VALUES --> %s' % mtc_values)
            # ---------------- DEBUG ----------------
        return

#--------------------------------------------------
# Main program
if __name__ == '__main__':
    try:
        rospy.loginfo('Executing robot topic subscriber')
        robot_topic = RobotTopicSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
