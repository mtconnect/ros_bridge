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
$: << './lib'
$: << './src'

require 'cnc'
require 'streamer'
require 'readline'
require 'optparse'

simulation = true
no_robot = false
machine_ip = '127.0.0.1'
robot_url = 'http://localhost:5000/Robot'
cnc_url = 'http://localhost:5000/cnc'

OptionParser.new do |opts|
  opts.banner = 'Usage: ruby cnc_simulator.rb [-sn] [-m machine_tool_ip] [robot_url] [cnc_url]'

  opts.on('-s', '--[no-]simulate', 'Simulation') do  |v|
    simulation = v
  end

  opts.on('-n', '--[no-]robot', 'Skip Robot Event Stream') do  |v|
    no_robot = v
  end

  opts.on('-m', '--cnc_ip <ip>', OptionParser::String, "IP (default: #{machine_ip})") do  |v|
    machine_ip = v
  end

  opts.parse!
  robot_url = ARGV.shift if ARGV.length > 0
  cnc_url = ARGV.shift if ARGV.length > 0
end

unless simulation
  # Connect to adapter on machine tool to control operations
  control = TCPSocket.new('127.0.0.1', 7879)
  Thread.new do
    while control.read(1024)

    end
  end
else
  control = nil
end

$context = Cnc::CncContext.new(control, 7879, simulation: simulation)
$context.statemachine.tracer = STDOUT
$context.start

unless no_robot
  $robot_streamer = MTConnect::Streamer.new(robot_url)
  robot_thread = $robot_streamer.start do |comp, name, value, code = nil, text = nil|
    begin
      $context.event('robot', comp, name, value, code, text)
    rescue
      puts "Error occurred in handling event: #{$!}"
      puts $!.backtrace.join("\n")
    end
  end
end

=begin
$cnc_streamer = MTConnect::Streamer.new(cnc_url,
                filter: '//DataItem[@type="CONTROLLER_MODE"or@type="EXECUTION"or@type="CHUCK_STATE"or@type="AVAILABILITY"or@category="CONDITION"]')
cnc_thread = $cnc_streamer.start do |comp, name, value, code = nil, text = nil|
  begin
    $context.event('cnc', comp, name, value, code, text)
  rescue
    puts "Error occurred in handling event: #{$!}"
    puts $!.backtrace.join("\n")
  end
end
=end

def parse_command(line)  
  res = 'success'
  
  case line.strip
  when "shutdown"
    $context.stop
    $robot_streamer.stop
    $cnc_streamer.stop
    exit(0)

  when "quit"
    res = 'quit'

  when "status"
    res = $context.status

  when /^events([ ]+\d+)?/i
    count = (($1 && $1.strip) || 10).to_i
    res = $context.events(count)

  when "help", "?"
    res = <<EOT
shutdown          - Stop the state machine
status            - Display state of state machines
events <n>        - Display last n events
event <source> <item> <value> [<alarm code>] [alarm text]
            - Send an event to the context
sm <event>           - Send an event to the cnc statemachine
^D or quit        - Stop this session
EOT

  when /event[ ]+(.+)$/
    args = $1.split(/[ ]+/, 5)
    $context.event(*args)

  when /^sm ([a-z_]+)$/i
    # State machine event
    event = $1.to_sym

    # send the line as an event...
    if $context.statemachine.respond_to? event
      $context.statemachine.send(event)
    else
      res = "CNC does not recognize #{event} in state #{$context.state}"
    end

  when /^ctx[ ]+(.+)$/i
    args = $1.split(/[ ]+/)
    meth = args.shift.to_sym
    args.map! { |a| eval(a) }
    if $context.respond_to? meth
      $context.send(meth, *args)
    else
      res = "CNC does not recognize #{meth}"
    end

  when /^fail[ ]+([a-z_]+)$/i
    $context.fail_next($1, true)

  else
    res = "Unrecognized command #{line.inspect}"
  end
  
  res
rescue
  puts $!, $!.backtrace.join("\n")
  "Error: #{$!}"
end

# Start a socket based command processor
Thread.new do
  server = TCPServer.new(3000)
  while true
    sock = server.accept
    Thread.new(sock) do |client|
      while true
        client.write("> ")
        client.flush

        line = client.readline
        if line.nil? or line.empty? or line[0] == "\04"
          break
        end

        res = parse_command(line)
        break if res == 'quit'
        client.puts res
        client.flush
      end
      client.close
    end
  end
end

while true
  line = Readline.readline('> ', true)
  puts parse_command(line)
end

robot_thread.join
cnc_thread.join
