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

import roslib; roslib.load_manifest('mtconnect_ros_bridge')
import rospy

import read_config_file
from importlib import import_module
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from mtconnect_adapter import Adapter

class RobotTopicSubscriber():
    def __init__(self):
        # Read configuration file and extract topics and types
        self.dataMap = read_config_file.obtain_dataMap()
        
        # Import topic type and MTConnect message members
        self.topic_name_list, self.mtc_dataitems = self.dataMap.keys(), self.dataMap.values()
        self.topic_type = []
        self.member_types = []
        self.member_names = []
        
        # Setup MTConnect Adapter
        rospy.loginfo('Create instance of Adapter')
        self.adapter = Adapter(('0.0.0.0', 7878))
        self.event = Event('close_chuck')
        self.adapter.add_data_item(self.event)
        self.avail = Event('avail')
        self.adapter.add_data_item(self.avail)
        self.avail.set_value('AVAILABLE')
        self.adapter.start()
        
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
        for val in self.topic_name_list:
            published_topics = dict(rospy.get_published_topics())
            if val in published_topics.keys():
                topic_type_string = published_topics[val]
                #print('topic_type_string --> %s' % topic_type_string) 
            else:
                print('WARNING: %s is not published and will not be included in the subscription' % val)
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
        # Assume topic type is in  "namespace/topic" form
        tokens = topic_type_string.split('/')
        namespace_string = tokens[0]
        type_string = tokens[1]
    
        # Create rostopic message module
        module_handle = __import__(namespace_string + '.msg')
        #print "Module '%s' handle contents: %s" % (module_handle.__name__, dir(module_handle))
        
        # Create list of module attributes with the given type_string
        type_handle = module_handle.msg.__dict__[type_string]
        
        # Return tuple of (module_name.msg, member_types, member names)
        return (namespace_string + '.msg', type_handle._slot_types, type_handle.__slots__)
    
    
    def topic_callback(self, data, cb_data):
        """Callback function that captures the attribute values for the topic.
        If the topic is a Robot REQUEST (i.e. REQUEST:OpenChuck), then an action client
        is executed.  The action client will then send the MTConnect tag via the 
        MTConnect adapter.  If the topic is a Robot RESPONSE (i.e. RESPONSE:MaterialLoad),
        only the updated XML tag is sent to the MTConnect adapter. Future work will include
        robot status messages that do not require a REQUEST or RESPONSE.
        This function is a work in progress.
        """
        self.lock.acquire()
        try:
            (topic_name, type_name, member_set, mtc_data) = cb_data
        
            # Repackage members into a dictionary
            dout = {val:val for val in member_set}
        
            # Create output string
            rospy.loginfo('Message on %s for %s' % (topic_name, rospy.get_name()))
            msg_data = []
            for s in member_set:
                if 'header' not in s:
                    # Acquire published data for the topic
                    attr_value = operator.attrgetter(dout[s])(data)
                    rospy.loginfo('%s: --> %s' % (s, attr_value))
                    msg_data.append(attr_value)

            # Push data to MTConnect adapter
            self.adapter.begin_gather()
            self.event.set_value(msg_data)
            self.adapter.complete_gather()
        except Exception as e:
            rospy.logerr('Topic callback failed: %s, releasing lock', e)
        finally:
            self.lock.release()
        return

    def topic_listener(self):
        """Main ROS subscriber function.  A new thread is created for each callback.
        self.topic_name_list and self.mtc_dataitems determined from the configuration file.
        The remaining callback data derived from the setup_topic_data function.
        """
        subscribers = []
        for each_topic, each_package, member_set, mtc_values in zip(self.topic_name_list, 
                                                                   self.topic_type, self.member_names, self.mtc_dataitems):
            rospy.init_node('rostopic_subscriber')
            subscribers.append(rospy.Subscriber(each_topic, each_package, self.topic_callback,
                                                (each_topic, each_package.__name__, member_set, mtc_values)))
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
