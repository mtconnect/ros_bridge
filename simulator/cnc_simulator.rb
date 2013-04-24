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
$: << './src'

require 'cnc'
require 'streamer'
require 'readline'
require 'optparse'

$simulation = false
no_robot = false
machine_ip = '192.168.1.69'
robot_url = 'http://localhost:5000/Robot'
cnc_url = 'http://localhost:5000/cnc'

OptionParser.new do |opts|
  opts.banner = 'Usage: ruby cnc_simulator.rb [-sn] [-m machine_tool_ip] [robot_url] [cnc_url]'

  opts.on('-s', '--[no-]simulate', 'Simulation') do  |v|
    $simulation = v
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

unless $simulation
  # Connect to adapter on machine tool to control operations
  control = TCPSocket.new('192.168.1.69', 7879)
  Thread.new do
    while control.read(1024)

    end
  end
else
  control = nil
end

context = Cnc::CncContext.new(control, 7879)
context.statemachine.tracer = STDOUT
context.start

unless no_robot
  robot_streamer = MTConnect::Streamer.new(robot_url)
  robot_thread = robot_streamer.start do |name, value, code = nil, text = nil|
    begin
      context.event('robot', name, value, code, text)
    rescue
      puts "Error occurred in handling event: #{$!}"
      puts $!.backtrace.join("\n")
    end
  end
end

cnc_streamer = MTConnect::Streamer.new(cnc_url,
                filter: '//DataItem[@type="CONTROLLER_MODE"or@type="EXECUTION"or@type="CHUCK_STATE"or@type="AVAILABILITY"or@category="CONDITION"]')
cnc_thread = cnc_streamer.start do |name, value, code = nil, text = nil|
  begin
    context.event('cnc', name, value, code, text)
  rescue
    puts "Error occurred in handling event: #{$!}"
    puts $!.backtrace.join("\n")
  end
end

# Start a socket based command processor
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

      begin
        case line.strip
        when "shutdown"
          context.stop
          robot_streamer.stop
          cnc_streamer.stop
          server.close
          exit(0)

        when "quit"
          break

        when "status"
          client.puts context.status

        when /^events([ ]+\d+)?/i
          count = (($1 && $1.strip) || 10).to_i
          client.puts context.events(count)

        when "help", "?"
          client.puts <<EOT
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
          context.event(*args)


        when /^sm ([a-z_]+)$/i
          # State machine event
          event = $1.to_sym

          # send the line as an event...
          if context.statemachine.respond_to? event
            context.statemachine.send(event)
          else
            client.puts "CNC does not recognize #{event} in state #{context.state}"
          end

        when /^ctx[ ]+(.+)$/i
          args = $1.split(/[ ]+/)
          meth = args.shift.to_sym
          args.map! { |a| eval(a) }
          if context.respond_to? meth
            context.send(meth, *args)
          else
            client.puts "CNC does not recognize #{meth}"
          end

        when /^fail[ ]+([a-z_]+)$/i
          context.fail_next($1, true)

        else
          client.puts "Unrecognized command #{line.inspect}"
        end
      rescue
        client.puts "Error: #{$!}"
        puts $!, $!.backtrace.join("\n")
      end
      client.flush
    end
    client.close
  end
end

while true
  begin
    line = Readline.readline('> ', true)

    if line.nil? or line.strip == "quit"
      context.stop
      robot_streamer.stop
      cnc_streamer.stop
      server.close
      exit 0
    end
    next if line.empty?
    event = line.strip.to_sym

    # send the line as an event...
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
