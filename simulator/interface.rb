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

module Cnc
  class Interface
    attr_accessor :statemachine
    attr_reader :interface, :state, :related
    include ThreadSafeStateMachine

    def initialize(adapter, interface, state, prefix, dest_state, transition_state, rel)
      @adapter, @interface, @state, @prefix, @dest_state, @transition_state = adapter, interface, state,
          prefix, dest_state, transition_state
      @related = nil
      @active = true
      self.related = rel if rel
    end

    def activate
      @active = true
      if  @statemachine.state == :not_ready or @statemachine.state == :fail
        @statemachine.ready
      end
    end

    def deactivate
      @active = false
      unless @statemachine.state == :not_ready or @statemachine.state == :fail
        @statemachine.not_ready
      end
    end

    def related=(rel)
      @related = rel
      rel.related = self unless rel.related
    end

    def not_ready
      @adapter.gather do
        @interface.value = 'NOT_READY'
      end
    end

    def ready
      if @active
        @adapter.gather do
          @interface.value = 'READY'
        end
      else
        @statemachine.not_ready
      end
    end

    def active
      puts "#{self.class} Active - #{@related.class} #{@related and @related.interface.value}"
      if @state == @dest_state
        @adapter.gather do
          @interface.value = 'ACTIVE'
        end
        @statemachine.complete
      elsif @related and (@related.interface.value == 'ACTIVE' or
        @related.interface.value == 'COMPLETE')
        @statemachine.fail
      else
        @adapter.gather do
          @interface.value = 'ACTIVE'
        end
      end
    end

    def complete
      puts "Completed"
      @adapter.gather do
        @interface.value = 'COMPLETE'
      end
    end


    def fail
      @adapter.gather do
        @interface.value = 'FAIL'
      end
      Thread.new do
        sleep 1
        @statemachine.not_ready
      end
    end

    def create_statemachine
      myself = self
      dest_event = "#{@prefix}_#{@dest_state.downcase}".to_sym
      trans_event = "#{@prefix}_#{@transition_state.downcase}".to_sym
      prefix = @prefix
      sm = Statemachine.build do
        startstate :base

        superstate :base do
          startstate :not_ready

          state :not_ready do
            on_entry :not_ready
            event :ready, :ready
            event :not_ready, :not_ready
            event "#{prefix}_open".to_sym, :not_ready
            event "#{prefix}_close".to_sym, :not_ready
            event "#{prefix}_unlatched".to_sym, :not_ready
            default :fail
          end

          state :ready do
            on_entry :ready
            event :active, :active
            event :unavailable, :not_ready
            event :not_ready, :not_ready
            event :fail, :fail
            default :ready
          end

          # Active state of interface
          state :active do
            on_entry :active
            default :fail
            event trans_event, :active
            event dest_event, :complete
            event :complete, :complete
          end

          # Handle invalid CNC state change in which we will respond with a fail
          # This requires CNC ack the fail with a FAIL. When we get the fail, we
          # transition to a ready
          state :fail do
            on_entry :fail
            default :fail
            event :fail, :not_ready
            event :not_ready, :not_ready
          end

          # These will auto transition to complete unless they fail.
          state :complete do
            on_entry :complete
            default :complete
            event :ready, :ready
            event :unavailable, :not_ready
            event :not_ready, :not_ready
          end
        end
        context myself
      end
      sm.tracer = STDOUT
      sm.name = self.class.name
    end
  end
end