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

class GenericActionClient():
    def __init__(self, action_name, action_members):
        self.client_name = action_name
        self.member_val = action_members
                
    def action_client(self):
        # Creates the SimpleActionClient, passing the type of the action (MaterialLoadAction) to the constructor.
        # i.e. client_name = 'MaterialLoadClient', action_type = mtconnect_msgs.msg.MaterialLoadAction
        co_str = 'action_type = mtconnect_msgs.msg.' + self.client_name[:-6] + 'Action'
        co_exec = compile(co_str, '', 'exec')
        exec(co_exec)
        client = actionlib.SimpleActionClient(self.client_name, action_type)
        
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Creates a MaterialLoad goal to send to the action server.
        co_str = 'goal = mtconnect_msgs.msg.' + self.client_name[:-6] + 'Goal()'
        co_exec = compile(co_str, '', 'exec')
        exec(co_exec)
        
        try:
            if set(self.member_val.values()) == set(goal.__slots__):
                for member, val in self.member_val.items():
                    co_str = 'goal.' + member + ' = ' + val
                    co_exec = compile(co_str, '', 'exec')
                    exec(co_exec)
                else:
                    rospy.logerr('Error: Member assignment not a valid action attribute')
        except Exception as e:
            rospy.logerr("ROS Action %s Client failed: %s, releasing lock" % (self.client_name, e))
        
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
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('ActionClient')
        result = material_load_client()
        rospy.loginfo('Action Result --> %s' % result)
    except rospy.ROSInterruptException:
        print 'program interrupted before completion'
        