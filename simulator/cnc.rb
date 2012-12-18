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
    attr_accessor :controller_mode, :material_load, :material_unload, :controller_mode
  
    def initialize(port = 7879)
      super(port)
      
      @adapter.data_items << (@availability_di = DataItem.new('avail'))

      @adapter.data_items << (@material_load_di = DataItem.new('material_load'))
      @adapter.data_items << (@material_unload_di = DataItem.new('material_unload'))

      @adapter.data_items << (@open_chuck_di = DataItem.new('open_chuck'))
      @adapter.data_items << (@close_chuck_di = DataItem.new('close_chuck'))

      @adapter.data_items << (@open_door_di = DataItem.new('open_door'))
      @adapter.data_items << (@close_door_di = DataItem.new('close_door'))


      @adapter.data_items << (@chuck_state_di = DataItem.new('chuck_state'))

      @adapter.data_items << (@link_di = DataItem.new('robo_link'))
      @adapter.data_items << (@exec_di = DataItem.new('exec'))
      @adapter.data_items << (@mode_di = DataItem.new('mode'))
      @adapter.data_items << (@door_state_di = DataItem.new('door_state'))

      @adapter.data_items << (@system_di = SimpleCondition.new('system'))

      @interfaces = [@open_door_di, @open_chuck_di, @close_door_di, @close_chuck_di,
                     @material_load_di, @material_unload_di]

      # Initialize data items
      @availability_di.value = "AVAILABLE"
      @chuck_state_di.value = 'UNLATCHED'
      @link_di.value = 'ENABLED'
      @exec_di.value = 'READY'
      @mode_di.value = 'AUTOMATIC'
      @door_state_di.value = "UNLATCHED"

      @interfaces.each { |i| i.value = 'NOT_READY' }

      @door_ready = false
      @load_ready = false
      @unload_ready = false
      @chuck_ready = false
      @controller_mode = false
      @material_load = false
      @material_unload = false
      
      @system_di.normal
      @adapter.start    
    end
  
    def stop
      @adapter.stop
    end
    
    def cnc_not_ready
      @adapter.gather do
        @interfaces.each { |i| i.value = 'NOT_READY' }
        @chuck_state_di.value = 'UNLATCHED'
        @door_state_di.value = 'UNLATCHED'
      end
    end

    def cnc_ready
      @adapter.gather do
        @interfaces.each { |i| i.value = 'READY' }
        @chuck_state_di.value = 'OPEN'
        @door_state_di.value = 'OPEN'
      end
      @statemachine.run
    end
    
    def activate
      if @faults.empty? and @mode_di.value == 'AUTOMATIC' and 
          @link_di.value == 'ENABLED' and @controller_mode == 'AUTOMATIC' and
          @material_load == 'READY'
        puts "Becomming operational"
        @statemachine.make_operational
      else
        puts "Still not ready"
        @statemachine.still_not_ready
      end
    end
    
    def automatic_mode
      @adapter.gather do
        @mode_di.value = 'AUTOMATIC'
      end
    end
    
    def manual_mode
      @adapter.gather do
        @mode_di.value = 'MANUAL'
      end
    end

    def enable
      @adapter.gather do
        @link_di.value = 'ENABLED'
      end
    end

    def disable
      @adapter.gather do
        @link_di.value = 'DISABLED'
      end
    end

    def running
      puts "****** Beginning New Part ******"

      @adapter.gather do
        @exec_di.value = 'READY'
        @material_load_di.value = 'READY'
        @material_unload_di.value = 'READY'
      end
      @statemachine.handling
    end

    def cycling
      @adapter.gather do
        @exec_di.value = 'ACTIVE'
        @material_load_di.value = 'READY'
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
        @exec_di.value = 'READY'
      end
    end

    def cycle_not_ready
      @cycle_ready = false
    end
    
    def cycle_ready
      @cycle_ready = true
    end
    
    def load_not_ready
      @load_ready = false
    end
    
    def load_ready
      @load_ready = true
    end

    def unload_not_ready
      @unload_ready = false
    end

    def unload_ready
      @unload_ready = true
    end

    def material_load
      @adapter.gather do
        @material_load_di.value = 'ACTIVE'
      end
    end

    def material_load_ready
      @adapter.gather do
        @material_load_di.value = 'READY'
      end
    end

    def material_unload
      @adapter.gather do
        @material_unload_di.value = 'ACTIVE'
      end
    end

    def material_unload_ready
      @adapter.gather do
        @material_unload_di.value = 'READY'
      end
    end


    [[:open_chuck, '@chuck_state_di', 'OPEN'], [:close_chuck, '@chuck_state_di', 'CLOSED'],
     [:open_door, '@door_state_di', 'OPEN'], [:close_door, '@door_state_di', 'CLOSED']].each do  |interface, state, dest|
      class_eval <<-EOT
        def #{interface}_begin
          @adapter.gather do
            @#{interface}_di.value = 'ACTIVE'
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
            @#{interface}_di.value = 'COMPLETE'
            #{state}.value = '#{dest}'
          end
        end

        def #{interface}_done
          @adapter.gather do
            @#{interface}_di.value = "READY"
          end
        end
      EOT
    end

    def one_cycle
      # Make sure spindle is not latched
      @adapter.gather do
        @load_material_di.value = 'READY'
      end
      Thread.new do
        sleep 1
        @statemachine.cycle_completed
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
      event :material_load_not_ready, :activated, :material_load_ready
      event :load_material_ready, :activated, :load_ready
      event :fault, :fault, :reset_history
      event :availability_unavailable, :activated

      # From the robot
      event :controller_mode_automatic, :activated
      event :material_load_ready, :activated

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
        event :controller_mode_manual, :activated, :reset_history

        
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
            event :material_load_active, :material_load
            event :material_load_complete, :cycle_start
            event :material_load_not_ready, :activated, :reset_history
          end

          state :material_unload do
            on_entry :material_unload
            event :material_unload_active, :material_unload
            event :material_unload_complete, :running
            event :material_unload_not_ready, :activated, :reset_history
          end
        end

        [:open_chuck, :close_chuck, :open_door, :close_door].each do |interface|
          active = "#{interface}_active".to_sym
          fail = "#{interface}_fail".to_sym
          failed = "#{interface}_failed".to_sym
          complete = "#{interface}_complete".to_sym
          completed = "#{interface}_completed".to_sym
          ready = "#{interface}_ready".to_sym
          cnc_fail = "cnc_#{interface}_fail".to_sym
          done = "#{interface}_done".to_sym

          trans :handling, active, interface

          # Active state of interface
          state interface do
            on_entry "#{interface}_begin".to_sym
            event ready, fail
            event complete, complete
            event fail, cnc_fail
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
          state cnc_fail do
            on_entry failed
            default cnc_fail
            event ready, :ready
          end

          event fail, cnc_fail

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