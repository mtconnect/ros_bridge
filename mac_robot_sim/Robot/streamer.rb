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

require 'net/http'
require 'long_pull'
require 'socket'
require 'rexml/document'
require 'time'

module MTConnect
  class Streamer
    def initialize(url)
      @url = url
      @reader = nil
    end
    
    def parse(xml, &block)
      nxt, instance = nil
      document = REXML::Document.new(xml)
      document.each_element('//Header') { |x| 
        nxt = x.attributes['nextSequence'].to_i 
        instance = x.attributes['instanceId'].to_i
      }
      
      events = []
      document.each_element('//Events/*') do |e|
        events << [e.attributes['sequence'].to_i, [e.name, e.text.to_s]]
      end
      document.each_element('//Condition/*') do |e|
        events << [e.attributes['sequence'].to_i, [e.name, "#{e.attributes['type']}_#{e.attributes['nativeCode']}"]]
      end
      
      events.sort.each { |e| block.call(*e[1]) }
      
      [nxt, instance] 
    end

    def current(client, path, &block)
      current = path.dup + 'current'
      resp = client.get(current)
      parse(resp.body, &block)
    end
    
    def stop
      @running = false
    end

    def start(&block)
      @running = true
      @reader = Thread.new do
        dest = URI.parse(@url)

        path = dest.path
        path += '/' unless path[-1] == ?/
        rootPath = path.dup

        begin
          puts "Connecting..."
          client = Net::HTTP.new(dest.host, dest.port)
          nxt, instance = current(client, rootPath, &block)
          
          path = rootPath + "sample?interval=0&count=1000&from=#{nxt}&heartbeat=1000"
          puller = LongPull.new(client)
          puller.long_pull(path) do |xml|
            nxt = parse(xml, &block)
            break unless @running
          end
          
        rescue 
          puts "Error occurred: #{$!}\n retrying..."
          puts $!.backtrace.join("\n")
          block.call('DISCONNECTED', nil)
          sleep 1
          
        rescue Exception
          puts "***** Exception: #{$!}"  
          exit 1        
        end while @running
      end
      @reader
    end
  end
end