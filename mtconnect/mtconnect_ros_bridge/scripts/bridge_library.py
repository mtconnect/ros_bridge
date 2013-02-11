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
#import operator
#import thread
import re
import time
import urllib2
#from Queue import Queue
#from threading import Thread, Timer
#from importlib import import_module
#from httplib import HTTPConnection
from xml.etree import ElementTree

# Import custom Python modules for MTConnect Adapter interface
path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')
from data_item import Event, SimpleCondition, Sample, ThreeDSample

# Import ROS Python modules
import rospy

def obtain_dataMap():
    """The code is executed with options, so this function
    utilizes python option parser to determine the option
    filename. Once the file name is obtained, the .yaml file
    contents are stored in a dictionary.  Program terminates
    if the option file is not available, or if it is in an
    incorrect .yaml format. 
    """
    def determine_config_file_name():
        parser = optparse.OptionParser()
        parser.add_option('-i', '--input',
                      dest="input_filename",
                      default=None,
                      )
        options, remainder = parser.parse_args()    
    
        if not options.input_filename:
            print('ERROR: Must provide .yaml configuration file')
            sys.exit(0)
        return options.input_filename if options.input_filename else None
    
    fn = determine_config_file_name()
    
    # Read file contents and store into dataMap dictionary
    try:
        with open(fn) as f:
            dataMap = yaml.load(f)
        if dataMap == {}:
            sys.exit(0)
    except IOError as e:
        print('({})'.format(e))
        sys.exit(0)
    return dataMap

def check_connectivity(data):
    # Unpack function arguments
    tout, url, url_port = data
    
    # Verify url availability, dwell until time out
    current = time.time()
    time_out = current + 20.0
    rospy.loginfo('Checking for URL availability')
    while time_out > current:
        try:
            response = urllib2.urlopen('http://' + url + ':' + str(url_port) + '/current', timeout = tout)
            rospy.loginfo('Connection available')
            break
        except urllib2.URLError as err:
            current = time.time()
            pass
    else:
        rospy.loginfo('System Time Out: URL Unavailable, check if the MTConnect Agent is running')
        sys.exit()
    return

def xml_get_response(data):
    # Unpack data
    url, url_port, port, conn, req = data
    
    # Get response from url    
    rospy.loginfo('Attempting HTTP connection on url: %s:%s\tPort:%s' % (url, url_port, port))
    conn.request("GET", req)
    response = conn.getresponse()
    if response.status != 200:
        rospy.loginfo("Request failed: %s - %d" % (response.reason, response.status))
        sys.exit(0)
    else:
        rospy.loginfo('Request --> %s, Status --> %s' % (response.reason, response.status))
    return response

def xml_components(xml, ns, tag_list, get_goal = False, action_goals = None):
    """ Find all elements in the updated XML.  root.find requires namespaces
    to be a dictionary.  Function returns two variables:
    1.  next XML sequence for streaming XML via longpull.py
    2.  only the elements that match the action items in the configuration file
    """
    
    # Extract XML Event elements
    root = ElementTree.fromstring(xml)
    header = root.find('.//m:Header', namespaces = ns)
    nextSeq = header.attrib['nextSequence']
   
    elements = []
    find_goal = None
    
    for tag, goals in tag_list.items():
        find_action = root.findall('.//m:' + tag, namespaces = ns)
        if find_action: # Element list is not empty
            elements.append(find_action[0])

        # Check if a goal must be captured
        if get_goal == True:
            goal_tag = goals.keys()[0]
            rospy.loginfo('GOAL-TAG --> %s' % goal_tag)
            find_goal = root.findall('.//m:' + goal_tag, namespaces = ns)
            rospy.loginfo('GOAL-ELEMENT --> %s' % find_goal)
            
            rospy.loginfo('ACTION --> %s' % tag)
            
            if find_goal:
                rospy.loginfo('GOAL-ELEMENT SET--> %s' % find_goal[0].text)
                goal_conv = []
                tokens = find_goal[0].text.split(", ")
                for item in tokens:
                    try:
                        goal_conv.append(float(item))
                    except ValueError:
                        goal_conv.append(item)
                action_goals[tag] = goal_conv
            else:
                rospy.loginfo('Action Goals --> %s' % action_goals)

    if get_goal == True:
        return nextSeq, elements, action_goals
    else:
        return nextSeq, elements 

def add_event(data):
    # Unpack function arguments
    adapter, tag_list, di_dict, init = data
    
    for tag in tag_list:
        # Change tag to XML [name] attribute format if necessary
        if not tag.islower():
            data_item = split_event(tag)
        else:
            data_item = tag
        
        # Add Event to the MTConnect adapter
        di_dict[data_item] = Event(data_item)
        adapter.add_data_item(di_dict[data_item])

        # Output success
        rospy.loginfo('Added XML data_item --> %s' % data_item)

    if init == True:
        # Set initial states for robot actions
        adapter.begin_gather()
        for data_item, event in di_dict.items():
            event.set_value('READY')
        adapter.complete_gather()
    return

def split_event(xml_tag):
    tokens = re.findall(r'([A-Z][a-z]*)', xml_tag)
    tokenlist = [val.lower() for val in tokens]
    data_item = tokenlist[0] + '_' + tokenlist[1]
    return data_item

def action_cb(data):
    # Unpack function arguments
    adapter, di_dict, data_item, state = data
    
    # Respond that goal is accepted
    #rospy.loginfo("Changing %s to '%s'" % (data_item, state))
    adapter.begin_gather()
    di_dict[data_item].set_value(state)
    adapter.complete_gather()
    return