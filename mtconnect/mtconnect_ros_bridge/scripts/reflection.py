#!/usr/bin/env python

def get_topic_variables(topic_type_string):
    """Given a topic type as an argument, this function
    extracts the message module types and attributes
    via introspection.
    """
    # Assume topic type is in  "namespace/topic" form
    tokens = topic_type_string.split('/')
    namespace_string = tokens[0]
    type_string = tokens[1]

    # Create rostopic message module
    module_handle = __import__(namespace_string + '.msg')
    #print "Module '%s' handle contents: %s" % (module_handle.__name__, dir(module_handle))
    
    # Create list of module attributes with the given type_string
    type_handle = module_handle.msg.__dict__[type_string]
    
    # Return tuple of (module_name.msg, member_types, member names)
    return (namespace_string + '.msg', type_handle._slot_types, type_handle.__slots__)
