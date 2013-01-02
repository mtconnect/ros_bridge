#!/usr/bin/env python

import sys
import optparse
import yaml

def obtain_dataMap():
    """The code is executed with options, so this function
    utilizes python option parser to determine the option
    filename. Once the file name is obtained, the .yaml file
    contents are stored in a dictionary.  Program terminates
    if the option file is not available, or if it is in an
    incorrect .yaml format. 
    """
    def determine_config_file_name():
        parser = optparse.OptionParser()
        parser.add_option('-i', '--input',
                      dest="input_filename",
                      default=None,
                      )
        options, remainder = parser.parse_args()    
    
        if not options.input_filename:
            print('ERROR: Must provide .yaml configuration file')
            sys.exit(0)
        return options.input_filename if options.input_filename else None
    
    fn = determine_config_file_name()
    
    # Read file contents and store into dataMap dictionary
    try:
        with open(fn) as f:
            dataMap = yaml.load(f)
    except IOError as e:
        print('({})'.format(e))
        sys.exit(0)
    return dataMap
