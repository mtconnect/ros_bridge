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

require 'mtc_context'

module Cnc
  class Request
    attr_accessor :statemachine, :fail_time_limit, :processing_time_limit
    attr_reader :interface, :related
    include ThreadSafeStateMachine

    def initialize(parent, adapter, interface, rel, simulate: false)
      @parent, @adapter, @interface = parent, adapter, interface

      @related = nil
      @active = true
      @simulate = simulate
      @failing = false

      @fail_time_limit = 1.0
      @processing_time_limit = 600.0

      @timer = nil

      self.related = rel if rel
    end

    def activate
      @statemachine.activate
    end

    def deactivate
      @statemachine.deactivate unless @failing
    end

    def idle
      @statemachine.idle
    end

    def state
      @statemachine.state
    end

    def start_timer(timeout)
      kill_timer

      @timer = Thread.new do
        sleep timeout
        @timer, timer = nil, @timer
        @statemachine.timeout if timer
      end
    end

    def kill_timer
      @timer, timer = nil, @timer
      Thread.kill(timer) if timer
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
      end
    end

    def processing
      start_timer @processing_time_limit
    end

    def complete
      @parent.completed(self)
      true
    end

    def failure
      @adapter.gather do
        @interface.value = 'FAIL'
      end
      start_timer(@fail_time_limit)
    end

    def complete_failed
      @failing = true
      kill_timer
      @parent.failed(self)

    ensure
      @failing = false
    end

    def create_statemachine
      myself = self

      sm = Statemachine.build do
        superstate :base do
          startstate :not_ready
          event :unavailable, :not_ready
          event :deactivate, :not_ready

          state :not_ready do
            on_entry :not_ready
            default :not_ready

            event :activate, :active
            event :idle, :ready
          end

          state :ready do
            on_entry :ready
            default :ready

            event :ready, :active
            event :activate, :active
          end

          state :active do
            on_entry :active
            default :active

            event :idle, :ready
            event :not_ready, :ready
            event :failure, :fail
            event :active, :processing
          end

          # When response is also active
          state :processing do
            on_entry :processing
            on_exit :kill_timer
            default :fail

            event :complete, :not_ready, :complete
          end

          state :fail do
            on_entry :failure
            on_exit :complete_failed
            default :ready
          end
        end

        context myself
      end
      sm.tracer = STDOUT
      sm.name = self.class.name
    end
  end
end