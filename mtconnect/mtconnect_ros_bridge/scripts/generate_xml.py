#!/usr/bin/env python

import sys

#from xml.etree.ElementTree import Element, SubElement, Comment, tostring, XML
from xml.etree import ElementTree
from xml.dom import minidom

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def data_to_xml(xml_data):
	# Establish root
	root = Element('root')

	# Iterate through the topics
	for topic_data in xml_data:
		topic_name = Element('topic', name=topic_data[0])
		topic_type = SubElement(topic_name, 'type', name=topic_data[1])
		topic_name.extend(topic_type)
	
		for member_val, msg in zip(topic_data[2], topic_data[3]):
			myattributes = {member_val:str(msg)}
			attributes = SubElement(topic_type, 'attributes', attrib=myattributes)
			topic_type.extend(attributes)

		root.extend(topic_name)

	print prettify(root)
	return prettify(root)

