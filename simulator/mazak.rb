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


require 'socket'
require 'net/http'
require 'readline'
require 'time'

puts "Waiting for connection..."

server = TCPServer.new((ARGV[0] || 7880).to_i)

loop do
  socket = server.accept
  Thread.new do
    while (select([socket], nil, nil))
      begin
        if (r = socket.read_nonblock(256)) =~ /\* PING/
          puts "Received #{r.strip}, responding with pong" if verbose
          mutex.synchronize {
            socket.puts "* PONG 10000"
          }
        else
          puts "Received '#{r.strip}'"
        end
      rescue
      end
    end
  end

  puts "Client connected"
  puts "Type an adapter feed string in the following format:"
  puts "> <key>|<value>"
  puts "> <key>|<value>|<key>|<value> ..."
  puts "> <alarm>|<code>|<native code>|<severity:CRITICAL|ERROR|WARNING|INFO>|<state:ACTIVE|CLEARED|INSTANT>|<message>"
  puts "> <condition>|<level:unavailable,normal,warning,fault>|<native code>|<native severity>|<qualifier>|<message>"
  loop do
    line = Readline::readline('> ')
    Readline::HISTORY.push(line)

    if line[0] == ?*
      puts "Writing line"
      socket.write "#{line}\n"
      socket.flush
    else
      ts = Time.now.utc
      stamp = "#{ts.iso8601[0..-2]}.#{'%06d' % ts.tv_usec}"
      puts "#{stamp}|#{line}"
      socket.write "#{stamp}|#{line}\n"
      socket.flush
    end

  end
end

