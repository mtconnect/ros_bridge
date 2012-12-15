"""Copyright 2012, System Insights, Inc.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License."""

import sys, os, time

path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/../src')

from data_item import Event, SimpleCondition, Sample, ThreeDSample
from mtconnect_adapter import Adapter

import roslib
roslib.load_manifest('turtlesim')
import rospy
import turtlesim.msg

class BridgeAdapter:
    def __init__(self):
        self._adapter = Adapter(('0.0.0.0', 7878))
        self._pose = ThreeDSample('pose')
        self._adapter.add_data_item(self._pose)

        avail = Event('avail')
        self._adapter.add_data_item(avail)
        avail.set_value('AVAILABLE')


    def start(self):
        def handle_turtle_pose(msg, turtlename):
                self._adapter.begin_gather()
                self._pose.set_value((msg.x, msg.y, 0.0))
                self._adapter.complete_gather()

        self._adapter.start()

        rospy.Subscriber('/turtle1/pose',
                         turtlesim.msg.Pose,
                         handle_turtle_pose,
                         'turtle1')



