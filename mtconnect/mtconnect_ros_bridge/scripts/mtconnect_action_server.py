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

import operator
import read_config_file # For .yaml configuration file

import roslib; roslib.load_manifest('mtconnect_msgs')
import rospy
import actionlib

import mtconnect_msgs.msg


class GenericActionServer():
    """DOCSTRING
    """

    def __init__(self):
        # Setup MTConnect to ROS Conversion
        self.dataMap = read_config_file.obtain_dataMap()
        #print('dataMap --> %s' % self.dataMap.keys())
        
        # Create a dictionary of _results to return upon action success
        self._resultDict = {}
        
        # Create a dictionary of action servers
        self._as = {}
        
        # Iterate through actions and create action call-backs and action result class instances
        for key in self.dataMap.keys():
            
            self.server_name = key
            
            # Create a dictionary of result class instances
            co_result_class_str = 'self._resultDict[key] = mtconnect_msgs.msg.' + key + 'Result()'
            co_result_class_expr = compile(co_result_class_str, '', 'exec')
            exec(co_result_class_expr)
            
            # Create instance of the action class
            co_action_msg = 'action_class = mtconnect_msgs.msg.' + key + 'Action'
            co_msg_expr = compile(co_action_msg, '', 'exec')
            exec(co_msg_expr)
            
            # Create action server for requested action
            co_actionlib_str = "self._as[key] = actionlib.SimpleActionServer(key + 'Client', action_class, self.execute_cb, False)"
            co_actionlib_expr = compile(co_actionlib_str, '', 'exec')
            exec(co_actionlib_expr)
            
            #rospy.loginfo('BEFORE CALLBACK_results = %s' % self._resultDict[key])
            
            self._as[key].start()
    
    def execute_cb(self, goal):
        key, request = goal.chuck_message, goal.chuck_request
        # Empty function -- assumes action was successful
        rospy.loginfo('In %s Callback -- determining action request result.  Request --> %s' % (key, request))
        
        # Extract action attribute
        result_attribute = self._resultDict[key].__slots__[0]
        
        # Create code object to set the attribute per the ROS to MTConnect conversion
        co_actionresult_str = 'self._resultDict[key].' + result_attribute + ' = self.dataMap[key][request]'
        co_actionresult_expr = compile(co_actionresult_str, '', 'exec')
        exec(co_actionresult_expr)
        
        # Indicate a successful action
        self._as[key].set_succeeded(self._resultDict[key])
        rospy.loginfo('In %s Callback -- action succeeded. Result --> %s' % (key, self.dataMap[key][request]))

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('ActionServer')
    rospy.loginfo('Started MTConnect Action Server')
    
    # Launch the action server
    server = MTConnect_Action()
    
    rospy.spin()
