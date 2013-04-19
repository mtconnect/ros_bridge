# Copyright 2012, System Insights, Inc.
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

$: << '.'

require 'cnc'
require 'streamer'
require 'readline'

context = Cnc::CncContext.new
context.statemachine.tracer = STDOUT
context.start

url = ARGV[0] || 'http://localhost:5000/Robot'
streamer = MTConnect::Streamer.new(url)
robot_thread = streamer.start do |name, value, code = nil, text = nil|
  begin
    context.event('robot', name, value, code, text)
  rescue
    puts "Error occurred in handling event: #{$!}"
    puts $!.backtrace.join("\n")
  end
end

url = ARGV[1] || 'http://localhost:5000/cnc'
streamer = MTConnect::Streamer.new(url,
                filter: '//DataItem[@type="CONTROLLER_MODE"or@type="EXECUTION"or@type="CHUCK_STATE"or@type="AVAILABILITY"or@category="CONDITION"]')
cnc_thread = streamer.start do |name, value, code = nil, text = nil|
  begin
    context.event('cnc', name, value, code, text)
  rescue
    puts "Error occurred in handling event: #{$!}"
    puts $!.backtrace.join("\n")
  end
end

# Command processor
while true
  begin
    line = Readline.readline('> ', true)    
    if line.nil? or line == 'quit'
      context.stop
      streamer.stop
      exit 0
    end
    next if line.empty?
    
    # send the line as an event...
    event = line.strip.to_sym
    if context.statemachine.respond_to? event
      context.statemachine.send(event)
    else
      puts "CNC does does not recognize #{event} in state #{Cnc.cnc.state}"
    end
  rescue
    puts "Error: #{$!}"
  end
end

robot_thread.join
cnc_thread.join
