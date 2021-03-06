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
  class Response
    attr_accessor :statemachine, :fail_reset_delay, :simulated_duration, :fail_next, :simulate
    attr_reader :interface, :state, :related
    include ThreadSafeStateMachine

    def initialize(parent, adapter, interface, prefix, dest_state, transition_state,
        rel, simulate: true)
      @adapter, @interface, @prefix, @dest_state,
        @transition_state = adapter, interface,
          prefix, dest_state, transition_state

      @parent = parent
      @related = nil
      @active = true
      @simulate = simulate
      @fail_reset_delay = 5.0
      @simulated_duration = 1.0
      @fail_next = false

      self.related = rel if rel
    end

    def reset
      @statemachine.reset
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
      puts "#{self.class} Active - #{@related.class} #{@related and @related.interface.value} #{response_state}"
      if @fail_next
        puts "**** Failing next action"
        @adapter.gather do
          @interface.value = 'ACTIVE'
        end
        @fail_next = false
        @statemachine.fail
      elsif response_state == @dest_state
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
          self.response_state = 'UNLATCHED' if @simulate
        end
        if @simulate
          Thread.new do
            sleep @simulated_duration
            @statemachine.complete
          end
        else
          execute
        end
      end
    end

    def response_state
      nil
    end

    def response_state=(value)
      nil
    end

    def execute

    end

    def complete
      puts "Completed"
      @adapter.gather do
        @interface.value = 'COMPLETE'
        self.response_state = @dest_state if @simulate
      end
      @parent.completed(self)
    end


    def failure
      @adapter.gather do
        @interface.value = 'FAIL'
      end
      @parent.failed(self)
      Thread.new do
        sleep @fail_reset_delay
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
            default :not_ready

            event :ready, :ready
            event :not_ready, :not_ready
            event "#{prefix}_open".to_sym, :not_ready
            event "#{prefix}_close".to_sym, :not_ready
            event "#{prefix}_unlatched".to_sym, :not_ready

            event :failure, :fail
            event :active, :fail
          end

          state :ready do
            on_entry :ready
            event :active, :active
            event :unavailable, :not_ready
            event :not_ready, :not_ready
            event :failure, :fail
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
            on_entry :failure
            default :fail
            event :failure, :not_ready
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