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

def material_load_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (MaterialLoadAction) to the constructor.
    #client = actionlib.SimpleActionClient('MaterialLoadClient', mtconnect_msgs.msg.MaterialLoadAction)
    client = actionlib.SimpleActionClient('ChuckAcknowledgeClient', mtconnect_msgs.msg.ChuckAcknowledgeAction)
    
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a MaterialLoad goal to send to the action server.
    #goal = mtconnect_msgs.msg.MaterialLoadGoal()
    #goal.material_length = 5.5
    #goal.material_diameter = 32.7
    #goal.material_message = 'MaterialLoad'
    #goal.material_request = 'LOADING'
    
    # Creates a ChuckAcknowledge goal to send to the action server.
    goal = mtconnect_msgs.msg.ChuckAcknowledgeGoal()
    goal.chuck_state = 'OPEN'
    goal.chuck_message = 'ChuckAcknowledge'
    goal.chuck_request = 'ACK'
    
    # Sends the goal to the action server.
    rospy.loginfo('Sending the goal')
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    rospy.loginfo('Waiting for result')
    client.wait_for_result()
    
    # Prints out the result of the executing action
    rospy.loginfo(('Returning the result --> %s' % client.get_result()))
    return client.get_result()


if __name__ == '__main__':
    try:
        # Setup MTConnect Adapter
        #self.adapter = Adapter(('0.0.0.0', 7878))
        #self.event = Event('close_chuck')
        #self.adapter.add_data_item(self.event)
        #self.avail = Event('avail')
        #self.adapter.add_data_item(self.avail)
        #self.avail.set_value('AVAILABLE')
        #self.adapter.start()
        
        
        
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('ActionClient')
        result = material_load_client()
        rospy.loginfo('Action Result --> %s' % result)
    except rospy.ROSInterruptException:
        print 'program interrupted before completion'
        