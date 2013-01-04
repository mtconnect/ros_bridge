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
    attr_accessor :statemachine
    include ThreadSafeStateMachine

    def initialize(adapter, interface, state, dest_state)
      @adapter, @interface, @state, @dest_state = adapter, interface, state, dest_state
    end

    def not_ready
      @adapter.gather do
        @interface.value = 'NOT_READY'
      end
    end

    def ready
      @adapter.gather do
        @interface.value = 'READY'
      end
    end

    def active
      @adapter.gather do
        @interface.value = 'ACTIVE'
        @state.value = 'UNLATCHED'
      end
      Thread.new do
        sleep 1
        @statemachine.complete
      end
    end

    def complete
      puts "Completed"
      @adapter.gather do
        @interface.value = 'COMPLETE'
        @state.value = @dest_state
      end
    end


    def fail
      @adapter.gather do
        @interface.value = 'FAIL'
      end
      Thread.new do
        sleep 1
        interface_done
      end
    end

    def create_statemachine
      myself = self
      @sm = Statemachine.build do
        startstate :base

        superstate :base do
          startstate :not_ready

          state :not_ready do
            on_entry :not_ready
            event :ready, :ready
            event :not_ready, :not_ready
            default :fail
          end

          state :ready do
            on_entry :ready
            event :active, :active
            event :unavailable, :not_ready
            event :fail, :fail
            default :ready
          end

          # Active state of interface
          state :active do
            on_entry :active
            event :ready, :fail
            event :unavailable, :fail
            event :complete, :complete
          end

          # Handle invalid CNC state change in which we will respond with a fail
          # This requires CNC ack the fail with a FAIL. When we get the fail, we
          # transition to a ready
          state :fail do
            on_entry :fail
            default :fail
            event :fail, :not_ready
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
      @sm.tracer = STDOUT
    end
  end
end