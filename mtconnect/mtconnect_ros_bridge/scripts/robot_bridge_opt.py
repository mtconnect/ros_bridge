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

import roslib; roslib.load_manifest('mtconnect_ros_bridge')
import rospy
import sys
import operator
import thread
from importlib import import_module

import read_config_file
import reflection
import generate_xml
import xml_post

class RobotTopicSubscriber():
    def __init__(self):
        # Read configuration file and extract topics and types
        self.dataMap = read_config_file.obtain_dataMap()
        
        # Import topic type and MTConnect message members
        self.topic_name_list, self.mtc_dataitems = self.dataMap.keys(), self.dataMap.values()
        self.topic_type = []
        self.member_types = []
        self.member_names = []
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
            self.member_types --> member type, not used in msg parameters
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
        
            #topic_type_string = reflection.get_topic_type(val) #Old use of the reflection.py file
        
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
        self.lock.acquire()
        try:
            (topic_name, type_name, member_set, mtc_data) = cb_data
        
            # Repackage members into a dictionary
            dout = {val:val for val in member_set}
        
            # Create output string
            rospy.loginfo('Message on %s for %s' % (topic_name, rospy.get_name()))
            msg_data = []
            for s in member_set:
                # Published data for the topic
                attr_value = operator.attrgetter(dout[s])(data)
                rospy.loginfo('%s: --> %s' % (s, attr_value))
                msg_data.append(attr_value)
            xml_data = [(topic_name, type_name, member_set, msg_data)]
            #print xml_data
            topic_xml = generate_xml.data_to_xml(xml_data)
            #mtc_xml = convert_xml_mtconnect.convert_xml(topic_xml, mtc_data)
            #xml_post.do_request(mtc_xml)
        except Exception as e:
            rospy.logerr('Topic callback failed: %s, releasing lock', e)
        finally:
            self.lock.release()
        return

    def topic_listener(self):
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


"""
#--------------------------------------------------
# Read configuration file and extract topics and types
dataMap = read_config_file.obtain_dataMap()

#--------------------------------------------------
# Import topic type and MTConnect message members
topic_name_list, mtc_dataitems = dataMap.keys(), dataMap.values()
#print 'MTConnect DataItems -->', mtc_dataitems
topic_type = []
member_types = []
member_names = []

for val in topic_name_list:
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

    #topic_type_string = reflection.get_topic_type(val)

    if topic_type_string:
        type_string = topic_type_string.split('/')[1]
    
        namespace_string, mb_types, mb_names = reflection.get_topic_variables(topic_type_string)
        member_types.append(mb_types)
        member_names.append(mb_names)
    
        # Import module, module name stored in relative terms
        topic_type.append(getattr(import_module(namespace_string), type_string)) # Python >= 2.7
    else:
        pass

#--------------------------------------------------
# Define the message callback function
def callback(data, cb_data):

    (topic_name, type_name, member_set, mtc_data) = cb_data

    # Repackage members into a dictionary
    dout = {val:val for val in member_set}

    # Create output string
    rospy.loginfo('Message on %s for %s' % (topic_name, rospy.get_name()))
    msg_data = []
    for s in member_set:
        # Published data for the topic
        attr_value = operator.attrgetter(dout[s])(data)
        rospy.loginfo('%s: --> %s' % (s, attr_value))
        msg_data.append(attr_value)
    xml_data = [(topic_name, type_name, member_set, msg_data)]
    #print xml_data
    topic_xml = generate_xml.data_to_xml(xml_data)
    #mtc_xml = convert_xml_mtconnect.convert_xml(topic_xml, mtc_data)
    #xml_post.do_request(mtc_xml)

#--------------------------------------------------
# Define the listener/subscriber function
def listener():
    #rospy.init_node('rostopic_subscriber')
    subscribers = []
    for each_topic, each_package, member_set, mtc_values in zip(topic_name_list, topic_type, member_names, mtc_dataitems):
        rospy.init_node('rostopic_subscriber --> ' + each_topic)
        subscribers.append(rospy.Subscriber(each_topic, each_package, callback, (each_topic, each_package.__name__, member_set, mtc_values)))

#--------------------------------------------------
# Main program
if __name__ == '__main__':
    listener()
    rospy.spin()

"""