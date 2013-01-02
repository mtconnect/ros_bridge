#!/usr/bin/env python

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
sys.path.append(os.path.realpath(path) + '/src')
   
from httplib import HTTPConnection
from xml.etree import ElementTree
from long_pull import LongPull
import thread
import time
import re
import operator

import read_config_file
import roslib
roslib.load_manifest('mtconnect_msgs')
import rospy
import mtconnect_msgs.msg

class MTConnectParser():
    def __init__(self):
        rospy.init_node('rostopic_relay')
        self.pub = rospy.Publisher('industrial_msgs/CncStatus', mtconnect_msgs.msg.CncStatus)
        self.sim_time = rospy.get_time()
        print self.sim_time
        self.stateDict = {}
        self.msg_statusDict = None
        
        # Setup MTConnect to ROS Conversion
        self.dataMap = read_config_file.obtain_dataMap()
        #print('dataMap --> %s' % self.dataMap.keys())
        
        # Establish localhost connection and read in XML
        conn = HTTPConnection('localhost', 5000)
        conn.request("GET", "/current")
        response = conn.getresponse()
        if response.status != 200:
            print "Request failed: %s - %d" % (response.reason, response.status)
            sys.exit(0)
        body = response.read()
        
        # Use XML to establish CNC state dictionary
        self.stateDict, self.events = self.setup_states(body)
        if not set(self.dataMap.keys()).issubset(set(self.events)):
            print 'ERROR: TOPIC CONFIG FILE HAS INCORRECT EVENTS'
            sys.exit(0)
        self.regexes = [re.compile(p) for p in self.events if p in self.dataMap.keys()]
        seq, _ = self.process_xml(body)

        conn.request("GET", "/sample?interval=1000&count=1000&from=" + seq)
        response = conn.getresponse()

        # Create class lock
        self.lock = thread.allocate_lock()

        # Create publishing thread
        rospy.Timer(rospy.Duration(0.1), self.ros_publisher)

        # Streams data from the agent...
        lp = LongPull(response)
        lp.long_pull(self.callback) # Runs until user interrupts

        
    def setup_states(self, xml):
        """Function that creates a dictionary of XML Tag:XML State pairs.
        This dictionary is used to track the status of states during runtime.
        A list of CNC Events is used by self.regexes in the regex_match function.
        """
        nextSeq, elements = self.xml_components(xml)
        stateDict = {e.tag:e.text for e in elements}
        cnc_events = []
        for key in stateDict.keys():
            value = re.findall(r'(?<=\})\w+',key)
            if value:
                cnc_events.append(value[0])
        return stateDict, cnc_events

    def xml_components(self, xml):
        """ Find all elements in the updated xml.
        root.find requires namespaces to be a dictionary.
        Return sequence and elements to process ROS msgs.
        """
        root = ElementTree.fromstring(xml)
        ns = dict(m = 'urn:mtconnect.org:MTConnectStreams:1.2')
        header = root.find('.//m:Header', namespaces=ns)
        nextSeq = header.attrib['nextSequence']
        elements = root.findall('.//m:Events/*', namespaces=ns)        
        return nextSeq, elements

    def regex_match(self, tag):
        """Searches through the set of compiled regular expressions for a
        match.  Returns the match if found (None if not) and a boolean.
        Events to be matched are stored in self.regexes.  The XML tag is
        updated by the CNC simulator.  Tags that haven't changed do not
        appear in the XML/current."""
        for regex in self.regexes:
            if regex.search(tag):
                return regex.pattern, True # Pattern returns only matched text
        return None, False

    def setup_statusDict(self):
        """Create a dictionary of key-->event_string : value-->[pos1,pos2]
        pos1 tracks if the event has changed.
        pos2 stores the event_string without the underscore: 'DoorState'
        """
        msg_statusDict = {}
        for key in self.dataMap.keys():
            textlist = re.findall(r'([A-Z][a-z]*)', key) # re to create list of words
            textlist = [val.lower() for val in textlist]
            event_str = textlist[0] + '_' + textlist[1] # create new event_string
            msg_statusDict[event_str] = [None, key]
        return msg_statusDict


    def process_xml(self, xml):        
        """Function determines if an event of interest has changed and updates
        the state dictionary and msg status dictionary.  Events that will be
        published are specified by the user in a .yaml file.  Since the MTConnect
        uses different tags for states, the function converts the tag from
        MTConnect to ROS.  The function returns the updated msg status dictionary
        that now includes the ROS attribute value for the ROS msg data structure.
        """
        nextSeq, elements = self.xml_components(xml)
        
        # Create event status dictionary
        local_statusDict = self.setup_statusDict()

        # Loop through XML elements, update the stateDict with the new state and
        # update the event status
        for e in elements:
            tag_val, tag_bool = self.regex_match(e.tag)
            if tag_bool: # Update msg state if event state has changed
                print ('Updating ROS message --> %s:\t%s --> %s --> %s at %4.2f' % (nextSeq, e.tag, 
                                                                                 e.text, tag_val, rospy.get_time() - self.sim_time))
                #self.sim_time = rospy.get_time()
                if tag_val in self.events:
                    # Loop through msg status dictionary and update states
                    for key, value in local_statusDict.items():
                        if value[1] == tag_val:
                            local_statusDict[key][0] = self.dataMap[tag_val][e.text]
                
                self.stateDict[e.tag] = e.text # Update dictionary with new state
        return nextSeq, local_statusDict

    def topic_publisher(self):
        """ Publishes the MTConnect message.  This function is a generic publisher.
        Attribute fetches are obtained via the attrgetter function in the operator
        module.  In order to assign a value to a relative attribute, a code object
        is created and then executed.  
        """
        msg = mtconnect_msgs.msg.CncStatus()
        msg.header.stamp = rospy.Time.now()
        
        for key, value in self.msg_statusDict.items():
            if value[0]: # Will be 'None' if the event did not change
                # Create callable object of the form 'door_state.OPEN'
                stateCall = operator.attrgetter(key + '.' + value[0])
                msg_value = stateCall(msg) # Obtain the attribute value

                # Create a code object and execute to assign val, i.e. msg.door_state.val = msg_value
                attrAssign_co = compile('msg.' + key + '.val = msg_value', '', 'exec')
                exec(attrAssign_co)
                #print('%s set from dataMap' % key.upper()) # DEBUG
            else:
                # If no change to the event, used stored value in stateDict.
                # Convert to ROS format via dataMap.
                #print('%s set from stateDict' % key.upper()) # DEBUG
                state_str = '{urn:mtconnect.org:MTConnectStreams:1.2}' + value[1]
                self.msg_statusDict[key][0] = self.dataMap[value[1]][self.stateDict[state_str]]

                # Obtain the state value via callable object: 'door_state.CLOSED'
                valueCall = operator.attrgetter(key + '.' + self.msg_statusDict[key][0])
                state_value = valueCall(msg)

                # Create a code object and execute to assign the attribute to the state value
                valAssign_co = compile('msg.' + key + '.val = state_value', '', 'exec')
                exec(valAssign_co)

        self.pub.publish(msg)
        return

    def callback(self, chunk):
        #print '*******************In PROCESS_XML callback***************'
        self.lock.acquire()
        try:
            _, self.msg_statusDict = self.process_xml(chunk)
        except Exception as e:
            rospy.logerr("Rostopic relay process XML callback failed: %s, releasing lock", e)        
        finally:
            self.lock.release()
        #print '*******************Done with PROCESS_XML callback***************'
        return
        
    def ros_publisher(self, event):
        #print '-------------------In ROS_PUBLISHER callback---------------'
        self.lock.acquire()
        try:
            if self.msg_statusDict != None:
                #print('self.msg_statusDict --> %s' % self.msg_statusDict) # DEBUG
                self.topic_publisher()
        except Exception as e:
            rospy.logerr("Rostopic relay publisher callback failed: %s, releasing lock", e)        
        finally:
            self.lock.release()
        #print '-------------------Done with ROS_PUBLISHER callback---------------'
        return

if __name__ == '__main__':
    try:
        rospy.loginfo('Publishing CNC Messages')
        mtc_parse = MTConnectParser()
    except rospy.ROSInterruptException:
        sys.exit(0)
