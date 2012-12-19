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

require 'rubygems'
require 'adapter'
require 'streamer'
require 'statemachine'
require 'readline'
require 'statemachine/generate/dot_graph'
require 'mtc_context'

module Cnc
  class CncContext < MTConnect::Context  
    include MTConnect
    attr_accessor :robot_controller_mode, :robot_material_load, :robot_material_unload, :robot_controller_mode,
                  :robot_open_chuck, :robot_close_chuck, :robot_open_door, :robot_close_door
  
    def initialize(port = 7879)
      super(port)
      
      @adapter.data_items << (@availability = DataItem.new('avail'))

      @adapter.data_items << (@material_load = DataItem.new('material_load'))
      @adapter.data_items << (@material_unload = DataItem.new('material_unload'))

      @adapter.data_items << (@open_chuck = DataItem.new('open_chuck'))
      @adapter.data_items << (@close_chuck = DataItem.new('close_chuck'))

      @adapter.data_items << (@open_door = DataItem.new('open_door'))
      @adapter.data_items << (@close_door = DataItem.new('close_door'))


      @adapter.data_items << (@chuck_state = DataItem.new('chuck_state'))

      @adapter.data_items << (@link = DataItem.new('robo_link'))
      @adapter.data_items << (@exec = DataItem.new('exec'))
      @adapter.data_items << (@mode = DataItem.new('mode'))
      @adapter.data_items << (@door_state = DataItem.new('door_state'))

      @adapter.data_items << (@material = DataItem.new('material'))


      @adapter.data_items << (@system = SimpleCondition.new('system'))

      @interfaces = [@open_door, @open_chuck, @close_door, @close_chuck,
                     @material_load, @material_unload]

      # Initialize data items
      @availability.value = "AVAILABLE"
      @chuck_state.value = 'OPEN'
      @link.value = 'ENABLED'
      @exec.value = 'READY'
      @mode.value = 'AUTOMATIC'
      @door_state.value = "OPEN"

      @material.value = 'ROUND 440C THING'

      @interfaces.each { |i| i.value = 'NOT_READY' }

      @system.normal
      @adapter.start    
    end
  
    def stop
      @adapter.stop
    end
    
    def cnc_not_ready
      @adapter.gather do
        @interfaces.each { |i| i.value = 'NOT_READY' }
      end
    end

    def cnc_ready
      @statemachine.run
    end
    
    def activate
      if @faults.empty? and @mode.value == 'AUTOMATIC' and
          @link.value == 'ENABLED' and @robot_controller_mode == 'AUTOMATIC' and
          @robot_material_load == 'READY' and @robot_material_unload == 'READY'
        puts "Becomming operational"
        @adapter.gather do
          @interfaces.each { |i| i.value = 'READY' }
        end
        @statemachine.make_operational
      else
        puts "Still not ready"
        @statemachine.still_not_ready
      end
    end
    
    def automatic_mode
      @adapter.gather do
        @mode.value = 'AUTOMATIC'
      end
    end
    
    def manual_mode
      @adapter.gather do
        @mode.value = 'MANUAL'
      end
    end

    def enable
      @adapter.gather do
        @link.value = 'ENABLED'
      end
    end

    def disable
      @adapter.gather do
        @link.value = 'DISABLED'
      end
    end

    def running
      puts "****** Beginning New Part ******"

      @adapter.gather do
        @exec.value = 'READY'
        @adapter.gather do
          @interfaces.each { |i| i.value = 'READY' }
        end
      end
      @statemachine.handling
    end

    def cycling
      @adapter.gather do
        @exec.value = 'ACTIVE'
        @material_load.value = 'READY'
      end
      Thread.new do
        puts "------ Cutting a part ------"
        sleep 5
        puts "------ Finished Cutting a part ------"
        @statemachine.cycle_complete
      end
    end

    def cycle_complete
      @adapter.gather do
        @exec.value = 'READY'
      end
    end

    def material_load
      @adapter.gather do
        @material_load.value = 'ACTIVE'
      end
    end

    def material_load_ready
      @adapter.gather do
        @material_load.value = 'READY'
      end
    end

    def material_unload
      @adapter.gather do
        @material_unload.value = 'ACTIVE'
      end
    end

    def material_unload_ready
      @adapter.gather do
        @material_unload.value = 'READY'
      end
    end

    [[:open_chuck, '@chuck_state', 'OPEN'], [:close_chuck, '@chuck_state', 'CLOSED'],
     [:open_door, '@door_state', 'OPEN'], [:close_door, '@door_state', 'CLOSED']].each do  |interface, state, dest|
      class_eval <<-EOT
        def #{interface}_begin
          @adapter.gather do
            @#{interface}.value = 'ACTIVE'
            #{state}.value = 'UNLATCHED'
          end
          Thread.new do
            sleep 1
            @statemachine.#{interface}_complete
          end
        end

        def #{interface}_completed
          puts "Completed"
          @adapter.gather do
            @#{interface}.value = 'COMPLETE'
            #{state}.value = '#{dest}'
          end
        end

        def #{interface}_done
          @adapter.gather do
            @#{interface}.value = "READY"
          end
        end

        def #{interface}_failed
          @adapter.gather do
            @#{interface}.value = 'FAIL'
          end
          Thread.new do
            sleep 1
            #{interface}_done
          end
        end
      EOT
    end

    def one_cycle
      # Make sure spindle is not latched
      @adapter.gather do
        @load_material.value = 'READY'
      end
      Thread.new do
        sleep 1
        @statemachine.cycle_completed
      end
    end

    def unload_failed
      @adapter.gather do
        @material_unload.value = 'FAIL'
      end
    end
    
    def reset_history
      @statemachine.get_state(:operational).reset
    end
  
    def add_conditions
    end  
  end

  @cnc = Statemachine.build do
    startstate :disabled
  
    superstate :base do  
      event :fault, :fault, :reset_history

      # From the robot
      event :robot_availability_unavailable, :activated
      event :robot_controller_mode_automatic, :activated
      event :robot_material_load_ready, :activated
      event :robot_material_load_not_ready, :activated
      event :robot_material_unload_not_ready, :activated
      event :robot_material_unload_ready, :activated


      # command lines
      event :auto, :activated, :automatic_mode
      event :maunal, :activated, :manual_mode
      event :disable, :activated, :disable
      event :enable, :activated, :enable
      event :normal, :activated

      superstate :disabled do
        default :not_ready
        default_history :not_ready
        on_entry :cnc_not_ready
    
        # Ways to transition out of not ready state...
        state :not_ready do
          on_entry :cnc_not_ready
          default :not_ready          
        end
  
        state :fault do
          on_entry :cnc_not_ready
          event :normal, :activated
        end
      end
  
      state :activated do
        on_entry :activate
        event :make_operational, :operational_H
        event :still_not_ready, :not_ready
      end
    
      superstate :operational do
        startstate :ready
        default_history :ready
        event :robot_controller_mode_manual, :activated, :reset_history
        event :robot_controller_mode_manual_data_input, :activated, :reset_history
        event :robot_material_unload_fail, :material_unload_failed
        
        state :ready do
          default :ready
          on_entry :cnc_ready
          event :run, :running
        end
        
        state :running do
          on_entry :running
          event :handling, :handling
        end

        state :cycle_start do
          on_entry :cycling
          event :cycle_complete, :material_unload, :cycle_complete
        end

        superstate :handling do
          default_history :material_load

          state :material_load do
            on_entry :material_load
            event :robot_material_load_active, :material_load
            event :robot_material_load_complete, :cycle_start
            event :robot_material_load_not_ready, :activated, :reset_history
          end

          state :material_unload do
            on_entry :material_unload
            event :robot_material_unload_active, :material_unload
            event :robot_material_unload_complete, :running
            event :robot_material_unload_not_ready, :activated, :reset_history
          end

          state :material_unload_failed do
            on_entry :unload_failed
            default :material_unload_failed
            event :robot_material_unload_fail, :activated, :reset_history
            event :robot_material_unload_ready, :activated, :reset_history
          end
        end

        [:open_chuck, :close_chuck, :open_door, :close_door].each do |interface|
          # Robot Events...
          active = "robot_#{interface}_active".to_sym
          fail = "robot_#{interface}_fail".to_sym
          ready = "robot_#{interface}_ready".to_sym
          robot_fail = "robot_#{interface}_fail".to_sym

          # CNC Events
          complete = "#{interface}_complete".to_sym

          # Intermediary states
          failed = "#{interface}_failed".to_sym
          completed = "#{interface}_completed".to_sym
          done = "#{interface}_done".to_sym

          trans :handling, active, interface

          # Active state of interface
          state interface do
            on_entry "#{interface}_begin".to_sym
            event ready, fail
            event complete, complete
            event fail, robot_fail
          end

          # Handle invalid CNC state change in which we will respond with a fail
          # This requires CNC ack the fail with a FAIL. When we get the fail, we
          # transition to a ready
          state fail do
            on_entry failed
            default fail
            event fail, :activated, :reset_history
          end

          # Handle CNC failing current operation. We should
          # only get here when we are operational and active.
          state robot_fail do
            on_entry failed
            default robot_fail
            event ready, :ready
          end

          event fail, robot_fail

          # These will auto transition to complete unless they fail.
          state complete do
            on_entry completed
            default complete
            event ready, :handling_H, done
          end
        end
      end
    end
    
    context CncContext.new    
  end

  def self.cnc
    @cnc
  end
end

if $0 == __FILE__
  module Statemachine
    module Generate
      module DotGraph
        class DotGraphStatemachine
          def explore_sm
            @nodes = []
            @transitions = []
            @sm.states.values.each { |state|
              state.transitions.values.each { |transition|
                @nodes << transition.origin_id
                @nodes << transition.destination_id
                @transitions << transition
              }
            }
            @transitions = @transitions.uniq
            @nodes = @nodes.uniq
          end
        end
      end
    end
  end
  dir = File.dirname(__FILE__) + '/graph'
  Dir.mkdir dir unless File.exist?(dir)
  Cnc.cnc.to_dot(:output => 'graph')
  Dir.chdir('graph') do
    system('dot -Tpng -o main.png main.dot')
    Dir['*.dot'].each do |f|
      system("dot -Tsvg -o #{File.basename(f, '.dot')}.svg #{f}")
    end
  end
end