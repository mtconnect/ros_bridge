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
    
import sys, os

path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/../src')
   
from httplib import HTTPConnection
from xml.etree import ElementTree
from long_pull import LongPull
import threading
import time

import roslib
roslib.load_manifest('turtlesim')
import rospy
import turtlesim.msg

msg1 = turtlesim.msg.Velocity(angular = 0.0, linear = 1.0)
msg2 = turtlesim.msg.Velocity(angular = 0.7853, linear = 0.0)

rospy.init_node('ntconnect_bridge')
pub = rospy.Publisher('/turtle1/command_velocity', turtlesim.msg.Velocity)

def spinner():
    rospy.spin()

threading.Thread(target = spinner)


def process_xml(xml):
    root = ElementTree.fromstring(xml)
    ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')
    header = root.find('.//m:Header', namespaces=ns)
    nextSeq = header.attrib['nextSequence']
    elements = root.findall('.//m:Events/*', namespaces=ns)
    for e in elements:
        print "%s: %s" % (e.tag, e.text)
        if e.tag == '{urn:mtconnect.org:MTConnectStreams:1.2}Execution' and \
          e.text != 'ACTIVE':
            pub.publish(msg1)
            time.sleep(1.0)
            pub.publish(msg2)
    return nextSeq

conn = HTTPConnection('agent.mtconnect.org')
conn.request("GET", "/current")
response = conn.getresponse()
if response.status != 200:
    print "Request failed: %s - %d" % (response.reason, response.status)
    exit()

body = response.read()
seq = process_xml(body)


conn.request("GET", "/sample?interval=1000&count=1000&from=" + seq)
response = conn.getresponse()

def callback(chunk):
    process_xml(chunk)

# Streams data from the agent...
lp = LongPull(response)
lp.long_pull(callback)

