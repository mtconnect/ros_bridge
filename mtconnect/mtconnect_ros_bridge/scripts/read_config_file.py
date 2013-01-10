#!/usr/bin/env python

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
