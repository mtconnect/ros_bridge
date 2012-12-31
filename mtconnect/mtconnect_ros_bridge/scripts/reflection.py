#!/usr/bin/env python

import rosgraph

def get_topic_type(topic_name):
    master = rosgraph.Master('reflection')
    topic_table = dict(master.getTopicTypes())
    return topic_table[topic_name]

def get_topic_variables(topic_type_string):
    # Assume topic type is in  "namespace/topic" form
    tokens = topic_type_string.split('/')
    namespace_string = tokens[0]
    type_string = tokens[1]

    # Get module handle
    module_handle = __import__(namespace_string + ".msg")
    # print "Module '%s' handle contents: %s" % (module_handle.__file__, dir(module_handle))
    
    # Get topic type handle
    type_handle = module_handle.msg.__dict__[type_string]

    # Return tuple of (member_types, member names)
    return (namespace_string + ".msg", type_handle._slot_types, type_handle.__slots__)
