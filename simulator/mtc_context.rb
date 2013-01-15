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


require 'adapter'
require 'statemachine'

module ThreadSafeStateMachine
  def statemachine=(statemachine)
    @statemachine = statemachine
    class << @statemachine
      @@event_mutex = Mutex.new
      @@thread = nil

      alias _process_event process_event

      # Make event processing thread safe
      def process_event(event, *args)
        if @@thread != Thread.current
          # puts "**** Waiting for lock ****"
          @@event_mutex.lock
          thread = @@thread = Thread.current
        end
        _process_event(event, *args)
      ensure
        if thread
          # puts "**** Releasing lock ****"
          @@thread = nil
          @@event_mutex.unlock
        end
      end
    end
  end
end

module MTConnect
  class Context
    attr_reader :statemachine
    include ThreadSafeStateMachine

    def initialize(port)
      @adapter = Adapter.new(port)
    
      @faults = {}
      @connected = false
    end
    
    def stop
      @adapter.stop
    end
    
    def event(name, value, code, text)
      puts "Received #{name} #{value} #{code} #{text}"
      case value
      when "Fault"
        key = "#{name}:#{code}"
        @faults[key] = true
        action = "robot_#{name.downcase}_#{value.downcase}".to_sym
        puts "    Trying action: #{action}"
        @statemachine.send(action) if @statemachine.respond_to? action
        @connected = true
    
      when 'Warning', 'Normal'
        if code.nil? or code.empty?
          @faults.delete_if { |k, v| k =~ /^#{name}/i }
        else
          key = "#{name}:#{code}"
          @faults.delete(key)
        end
        action = "robot_#{name.downcase}_#{value.downcase}".to_sym
        puts "    Trying action: #{action}"
        @statemachine.send(action) if @statemachine.respond_to? action
        @connected = true

      when 'Unavailable'
        action = "robot_#{name.downcase}_#{value.downcase}".to_sym
        @statemachine.send(action) if @statemachine.respond_to? action

      when 'DISCONNECTED'
        @statemachine.disconnected
        @connected = false
        @adapter.gather do 
          add_conditions
        end
    
      else
        @connected = true

        # Convert camel case to lower _ separated words: FizzBangFlop fizz_bang_flop
        element = name.split(/([A-Z][a-z]+)/).delete_if(&:empty?).map(&:downcase).join('_')
        mth = "robot_#{element}=".to_sym
        puts "    Trying method: #{mth} #{value}"
        self.send(mth, value) if self.respond_to? mth
    
        # Only send valid events to the statemachine.
        action = "robot_#{element}_#{value.downcase}".to_sym
        puts "    Trying action: #{action}"
        @statemachine.send(action) if @statemachine.respond_to? action
      end
    end
  end
end