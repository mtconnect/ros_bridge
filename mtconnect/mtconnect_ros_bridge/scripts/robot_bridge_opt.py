#!/usr/bin/env python

import roslib; roslib.load_manifest('mtconnect_ros_bridge')
import rospy
import sys
import operator
import optparse
import yaml
from importlib import import_module
import reflection
import generate_xml
import xml_post

#--------------------------------------------------
# Read configuration file and extract topics and types
#
def determine_config_file_name():
    parser = optparse.OptionParser()
    parser.add_option('-i', '--input',
                  dest="input_filename",
                  default=None,
                  )
    options, remainder = parser.parse_args()    
    #print 'INPUT     :', options.input_filename
    if not options.input_filename:
        print('ERROR: Must provide configuration file')
        sys.exit(0)
    return options.input_filename if options.input_filename else None

fn = determine_config_file_name()

# Store data from .yaml file to a dictionary, keys: Topic, Type
try:
    with open(fn) as f:
        dataMap = yaml.load(f)
except IOError as e:
    print('({})'.format(e))
    sys.exit(0)

#--------------------------------------------------
# Import topic type and MTConnect message members
topic_name_list, mtc_dataitems = dataMap.keys(), dataMap.values()
print 'MTConnect DataItems -->', mtc_dataitems
topic_type = []
member_types = []
member_names = []

for val in topic_name_list:
    topic_type_string = reflection.get_topic_type(val)
#    print topic_type_string

    type_string = topic_type_string.split('/')[1]

    namespace_string, mb_types, mb_names = reflection.get_topic_variables(topic_type_string)

    member_types.append(mb_types)
    member_names.append(mb_names)

    # Import module, module name stored in relative terms
    topic_type.append(getattr(import_module(namespace_string), type_string)) # Python >= 2.7

#--------------------------------------------------
# Define the message callback function
def callback(data, cb_data):

    (topic_name, type_name, member_set, mtc_data) = cb_data

    # Repackage members into a dictionary
    dout = {val:val for val in member_set}

    # Create output string
    rospy.loginfo('Message on %s for %s' % (topic_name, rospy.get_name()))
    msg_data = []
    for s in member_set:
        # Published data for the topic
        rospy.loginfo('%s: --> %s' % (s, operator.attrgetter(dout[s])(data)))
        msg_data.append(operator.attrgetter(dout[s])(data))
    xml_data = [(topic_name, type_name, member_set, msg_data)]
    topic_xml = generate_xml.data_to_xml(xml_data)
    mtc_xml = convert_xml_mtconnect.convert_xml(topic_xml, mtc_data)
    #xml_post.do_request(mtc_xml)

#--------------------------------------------------
# Define the listener/subscriber function
def listener():
    rospy.init_node('listener', anonymous=True)
    subscribers = []
    for each_topic, each_package, member_set, mtc_values in zip(topic_name_list, topic_type, member_names, mtc_dataitems):
        subscribers.append(rospy.Subscriber(each_topic, each_package, callback, (each_topic, each_package.__name__, member_set, mtc_values)))
    rospy.spin()

#--------------------------------------------------
# Main program
if __name__ == '__main__':
    listener()

