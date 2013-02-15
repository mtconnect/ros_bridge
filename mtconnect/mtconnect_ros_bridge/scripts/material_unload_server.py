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

import sys
import logging
import threading
import time
import collections
from httplib import HTTPConnection
from xml.etree import ElementTree

import roslib; roslib.load_manifest('mtconnect_msgs')
import rospy
import actionlib
import mtconnect_msgs.msg


class MaterialUnloadServer():
    """Dedicated Material Unload Server -- without robot interface
    Class establishes a simple action server for the MaterialUnloadAction, and
    starts a thread to subscribe to the ROS topic CncStatus.
    
    The Material Unload sequence is completed once the door state and the chuck
    state are open.
    
    @param server_name: string, 'MaterialUnload'
    @param _result: instance of the mtconnect_msgs.msg.MaterialUnloadResult class
    @param _as: ROS actionlib SimpleActionServer
    @param door_state: int, published value for the CNC door state [0:Open, 1:Closed, -1:Unlatched]
    @param chuck_state: int, published value for the CNC chuck state [0:Open, 1:Closed, -1:Unlatched]
    @param sub_thread: thread that launches a ROS subscriber to the CncResponseTopic via bridge_publisher node

    """

    def __init__(self):
        self.server_name = 'MaterialUnload'
        
        self._result = mtconnect_msgs.msg.MaterialUnloadResult()
        self._as = actionlib.SimpleActionServer('MaterialUnloadClient', mtconnect_msgs.msg.MaterialUnloadAction, self.execute_cb, False)
        self._as.start()
        self._as.accept_new_goal()
        
        # Subscribe to CNC state topic
        self.door_state = None
        self.chuck_state = None
        self.open_chuck = None
        
        # Check for CncResponseTopic
        dwell = 2
        while True:
            published_topics = dict(rospy.get_published_topics())
            if '/CncResponseTopic' in published_topics.keys():
                rospy.loginfo('ROS CncResponseTopic available, starting subscriber')
                break
            else:
                rospy.loginfo('ROS CncResponseTopic not available, will try to subscribe in %d seconds' % dwell)
                time.sleep(dwell)
        
        # Create ROS Subscriber thread
        sub_thread = threading.Thread(target = self.subscriber_thread)
        sub_thread.daemon = True
        sub_thread.start()
    
    def execute_cb(self, goal):
        rospy.loginfo('In %s Bridge Server Callback -- determining action request result.' % self.server_name)
        
        # Initialize timeout parameter
        start = time.time()
        
        # Start while loop and check for cnc action changes
        rospy.loginfo('In MaterialUnload Server while loop')
        previous = time.time()
        dwell = True
        while dwell == True:
            try:
                # Current CNC state
                if time.time() - previous > 1.0:
                    rospy.loginfo('CNC States [door_state, chuck_state, open_chuck]: %s' % [self.door_state, self.chuck_state, self.open_chuck])
                    previous = time.time()
    
                if self.door_state == 0 and self.chuck_state == 0 and self.open_chuck == 0:
                    # Chuck and Door are closed, complete the material load cycle
                    self._result.unload_state = 'COMPLETE'
                    dwell = False
                    rospy.loginfo('CNC States [door_state, chuck_state, open_chuck]: %s' % [self.door_state, self.chuck_state, self.open_chuck])
    
                # Check for timeout
                if time.time() - start > 120.0:
                    rospy.loginfo('Material Unload Server Timed Out')
                    sys.exit()
            except rospy.ROSInterruptException:
                rospy.loginfo('program interrupted before completion')
        
        # Indicate a successful action
        self._as.set_succeeded(self._result)
        rospy.loginfo('In %s Callback -- action succeeded. Result --> %s' % (self.server_name, self._result.unload_state))
        return self._result
    
    def topic_callback(self, msg):
        self.door_state = msg.door_state.val
        self.chuck_state = msg.chuck_state.val
        self.open_chuck = msg.open_chuck.val
        return
    
    def subscriber_thread(self):
        rospy.Subscriber('CncResponseTopic', mtconnect_msgs.msg.CncStatus, self.topic_callback)
        rospy.spin()
        return

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('MaterialUnloadServer')
    rospy.loginfo('Started ROS MaterialUnload Server')
    
    # Launch the action server
    server = MaterialUnloadServer()
    rospy.spin()
