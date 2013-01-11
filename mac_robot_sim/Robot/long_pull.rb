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
require 'strscan'

class LongPull
  class Timeout < RuntimeError; end
  
  def initialize(http)
    @http = http
  end

  def long_pull(uri, timeout = 2.0, &block)
    @http.request_get(uri) do |res|
      content_type, = res.get_fields('content-type')
      if content_type !~ /boundary=([^;]+)/
        raise "No boundry: #{res.body}"
      end
      
      boundary = "--#{$1}"
      document = StringScanner.new(String.new)
      header = true
      length = boundary.length
      
      # Create a timeout thread to kill the read in case the agent becomes unresponsive...
      ct = Thread.current
      chunk_time = Time.now
      tot = Thread.start {
        while true
          nap = timeout - (Time.now - chunk_time)
          if nap > 0.0
            sleep nap 
          else
            ct.raise Timeout, 'Read timed out'
          end
        end
      }
      
      begin
        res.read_body do |chunk|
          next if chunk.empty?
          document << chunk

          while document.rest_size >= length
            if header
              if !document.check(/^#{boundary}/)
                puts document.rest
                raise "Framing error"
              end
                        
              break unless head = document.scan_until(/\r\n\r\n/)
              bound, *rest = head.split(/\r\n/)
            
              fields = Hash[*rest.map { |s| s.split(/:\s*/) }.flatten]
              length = fields['Content-length'].to_i
              header = false
            else
              # We have now received a complete chunk from the server, so we'll mark time here to 
              # make the timeout thread happy.
              chunk_time = Time.now
              rest = document.rest
            
              # Slice off the chunk we need
              body = rest[0, length]
            
              document.clear
              document = StringScanner.new(rest[length..-1])
              
              block.call(body)
              # puts body
            
              # New message
              length = boundary.length
              header = true
            end
          end
        end
      ensure
        tot.kill
        # Make sure it exited, don't want it killing anything else...
        tot.join
      end
    end
  end
end
