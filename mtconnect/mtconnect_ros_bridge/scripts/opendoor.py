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

import roslib; roslib.load_manifest('mtconnect_msgs')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the material_load action.
import mtconnect_msgs.msg

def open_door_client():
    rospy.loginfo('Launched OpenDoor Action CLient')
    # Creates the SimpleActionClient, passing the type of the action
    # (OpenDoorAction) to the constructor.
    client = actionlib.SimpleActionClient('OpenDoorClient', mtconnect_msgs.msg.OpenDoorAction)
    
    # Waits until the action server has started up and started listening for goals.
    rospy.loginfo('Waiting for Generic Action Server')
    client.wait_for_server()
    rospy.loginfo('Generic Action Server Activated')

    # Creates a DoorAcknowledge goal to send to the action server.
    goal = mtconnect_msgs.msg.OpenDoorGoal()
    goal.open_door = 'CLOSED'
    
    # Sends the goal to the action server.
    rospy.loginfo('Sending the goal')
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    rospy.loginfo('Waiting for result')
    client.wait_for_result()
    
    # Obtain result
    result = client.get_result() # result must be a string
    rospy.loginfo('Returning the result --> %s' % result)
    return

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('OpenDoorActionClient')
        result = open_door_client()
        rospy.loginfo('Action Result --> %s' % result)
    except rospy.ROSInterruptException:
        print 'program interrupted before completion'

