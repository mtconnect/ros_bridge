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
        rospy.init_node('bridge_publisher')
                
        # Setup MTConnect to ROS Conversion
        self.config = read_config_file.obtain_dataMap()
        #rospy.loginfo('config file --> %s' % self.config.keys())
        
        # Create the data sets for the topic types, member names, and member types
        self.setup_topic_data()
        
        """
        # Setup ROS publisher and topic hash table keys
        for topic, topic_type in self.config.items():
            # TOPIC --> topic name, such as 'chatter'
            # TOPIC TYPE --> message namespace and name, such as mtconnect_msgs/CncStatus
            tokens = topic_type.keys()[0].split('/')
            namespace = tokens[0]
            type_name = tokens[1]
            
            # Import module and create topic type class, called pub_handle
            #    i.e. append <class 'mtconnect_msgs.msg._RobotStates.RobotStates'>
            rospy.loginfo('Class Instance --> ' + namespace + '.msg.' + type_name)
            
            roslib.load_manifest(namespace)
            
            pub_handle = getattr(import_module(namespace + '.msg'), type_name)
            
            # Create ROS publisher
            self.pub = rospy.Publisher(topic, pub_handle)
            
            # Instance of topic type message class
            self.msg = pub_handle()
            
            # Capture topic name and type for dataMap reference
            self.topic_name = topic
            self.topic_type = topic_type.keys()[0] # Only one type per topic
            
            # Store the data items for this topic
            self.data_items = [data_item for data_item in topic_type[self.topic_type].keys()]
        """     
        # Hash table of persistent CNC event states, only updates when state changes
        self.di_current = {}
        
        # Hash table to store changed CNC event states
        self.di_changed = None
        
        # With the CNC localhost connection, read in current CNC XML
        conn = HTTPConnection('localhost', 5000)
        conn.request("GET", "/cnc/current")
        response = conn.getresponse()
        if response.status != 200:
            rospy.loginfo("Request failed: %s - %d" % (response.reason, response.status))
            sys.exit(0)
        cnc_body = response.read()
        
        # Parse the XML and determine the current sequence
        seq, elements = self.xml_components(cnc_body)
        
        # Use XML to establish CNC state dictionary
        self.di_current = {e.attrib['name']:e.text for e in elements if 'name' in e.attrib.keys()}
        
        # Confirm that data items in config file are present in XML
        if not set(self.data_items).issubset(set(self.di_current.keys())):
            rospy.logerr('ERROR: INCORRECT EVENTS IN TOPIC CONFIG OR XML AGENT FILE')
            sys.exit()
        
        # Start a streaming XML connection
        conn.request("GET", "/cnc/sample?interval=1000&count=1000&from=" + seq)
        response = conn.getresponse()
        #---------------------------DEBUG------------------
        #for key, val in self.di_current.items():
        #    rospy.loginfo('CNC KEY --> %s\tVAL --> %s' % (key, val))
        #---------------------------DEBUG------------------
        
        # Create class lock
        self.lock = thread.allocate_lock()

        # Create publishing thread
        rospy.Timer(rospy.Duration(0.1), self.ros_publisher)

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
        
            self.topic_type_list   --> used for module import and msg parameters
            self.member_types --> member type, not used in msg parameters, future use
            self.member_names --> used for ROS subscriber msg parameters
        """
        rospy.loginfo('TOPIC NAME LIST --> %s' % self.topic_name_list)
        
        for topic_name, type_name in zip(self.topic_name_list, self.subscribed_list):
            # TOPIC --> topic name, such as 'chatter'
            # TOPIC TYPE --> message namespace and name, such as mtconnect_msgs/CncStatus
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

            #self.topic_type_list.append(type_handle)
            #self.msg_text.append(type_handle._full_text)
            #self.member_types.append(type_handle._slot_types)
            #self.member_names.append(type_handle.__slots__)
            self.data_items.append([val for val in type_handle.__slots__ if val != 'header'])
            
            # Create ROS publisher
            self.pub.append(rospy.Publisher(topic_name, type_handle))
            
            # Instance of topic type message class
            self.msg.append(type_handle())
            
            # Capture topic name and type for dataMap reference
            self.topic_name = topic_name
            self.topic_type = topic_type.keys()[0] # Only one type per topic
            
            
        return

    def xml_components(self, xml):
        """ Find all elements in the updated xml.
        root.find requires namespaces to be a dictionary.
        Return sequence and elements to process ROS msgs.
        """
        #rospy.loginfo('XML STREAM -->\n%s' % xml)
        root = ElementTree.fromstring(xml)
        ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')
        header = root.find('.//m:Header', namespaces=ns)
        nextSeq = header.attrib['nextSequence']
        elements = root.findall('.//m:Events/*', namespaces=ns)        
        return nextSeq, elements
    
    def init_di_dict(self):
        return {item:None for item in self.data_items}
    
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
            
            if 'name' in e.attrib.keys():
                if e.attrib['name'] in self.data_items:
                    # Change local_di from None to current MTConnect event status
                    di = e.attrib['name']
                    local_di[di] = e.text
                    self.di_current[di] = e.text # Update di_current with new state
        return nextSeq, local_di

    def topic_publisher(self):
        """ Publishes the MTConnect message.  This function is a generic publisher.
        Attribute fetches are obtained via the attrgetter function in the operator
        module.  In order to assign a value to a relative attribute, a code object
        is created and then executed.  
        """
        self.msg.header.stamp = rospy.Time.now()
        
        for di_name, value in self.di_changed.items():
            # Convert tag text from MTConnect to ROS
            if value is not None:
                # Used changed value in di_changed
                rospy.loginfo('%s changed, set from di_changed' % di_name) # DEBUG
                
                # Check for conversion key error
                conv_keys = self.config[self.topic_name][self.topic_type][di_name].keys()
                if value not in conv_keys:
                    rospy.logerr("CONVERSION ERROR IN TOPIC CONFIG FILE: XML is '%s' --> CONFIG is %s" % (value, conv_keys))
                    os._exit(0)
                
                # Convert from MTConnect to ROS format from di_changed
                ros_tag = self.config[self.topic_name][self.topic_type][di_name][self.di_changed[di_name]]
            else:
                # Use stored value in di_current
                #rospy.loginfo('%s did not change, set from di_current' % di_name) # DEBUG
                
                # Convert from MTConnect to ROS format via di_current
                ros_tag = self.config[self.topic_name][self.topic_type][di_name][self.di_current[di_name]]

            # Obtain the state value via callable object: 'door_state.CLOSED'
            valueCall = operator.attrgetter(di_name + '.' + ros_tag)
            state_value = valueCall(self.msg)

            # Create a code object and execute to assign the attribute to the state value
            co_str = compile('self.msg.' + di_name + '.val = state_value', '', 'exec')
            exec(co_str)

        self.pub.publish(self.msg)
        return

    def xml_callback(self, chunk):
        #rospy.loginfo('*******************In PROCESS_XML callback***************')
        self.lock.acquire()
        try:
            _, self.di_changed = self.process_xml(chunk)
        except Exception as e:
            rospy.logerr("Rostopic relay process XML callback failed: %s, releasing lock" % e)
        finally:
            self.lock.release()
        #rospy.loginfo('*******************Done with PROCESS_XML callback***************')
        return
        
    def ros_publisher(self, event):
        #rospy.loginfo('-------------------In ROS_PUBLISHER callback---------------')
        self.lock.acquire()
        try:
            if self.di_changed != None:
                # Publish topic
                self.topic_publisher()
                
                # Reset current data items to None
                self.di_changed = self.init_di_dict()
        except Exception as e:
            rospy.logerr("Rostopic relay publisher callback failed: %s, releasing lock" % e)        
        finally:
            self.lock.release()
        #rospy.loginfo('-------------------Done with ROS_PUBLISHER callback---------------')
        return

if __name__ == '__main__':
    try:
        rospy.loginfo('Publishing CNC Messages')
        mtc_parse = BridgePublisher()
    except rospy.ROSInterruptException:
        sys.exit(0)
