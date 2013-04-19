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
require 'statemachine'
require 'statemachine/generate/dot_graph'
require 'mtc_context'
require 'chuck'
require 'door'
require 'material'

module Cnc
  class CncContext < MTConnect::Context  
    include MTConnect
    attr_accessor :robot_controller_mode, :robot_material_load, :robot_material_unload, :robot_controller_mode,
                  :robot_open_chuck, :robot_close_chuck, :robot_open_door, :robot_close_door,
                  :robot_execution, :robot_availability

    attr_accessor :cnc_controller_mode, :cnc_execution, :cnc_availability, :cnc_chuck_state

    attr_reader :adapter, :open_chuck, :close_chuck, :door_state, :open_door, :close_door,
                :material_load, :material_unload, :link, :system, :has_material

    attr_accessor :cycle_time, :has_material


    def initialize(port = 7879)
      super(port)

      @adapter.data_items << (@availability = DataItem.new('avail'))

      @adapter.data_items << (@material_load = DataItem.new('material_load'))
      @adapter.data_items << (@material_unload = DataItem.new('material_unload'))

      @adapter.data_items << (@open_chuck = DataItem.new('open_chuck'))
      @adapter.data_items << (@close_chuck = DataItem.new('close_chuck'))

      @adapter.data_items << (@open_door = DataItem.new('open_door'))
      @adapter.data_items << (@close_door = DataItem.new('close_door'))

      @adapter.data_items << (@link = DataItem.new('robo_link'))

      @adapter.data_items << (@material = DataItem.new('material'))

      # CNC Data items not supplied by the machine tool adapter
      @adapter.data_items << (@system = SimpleCondition.new('infc_system'))

      # Controller does not have a door signal, so we need to simulate it here.
      @adapter.data_items << (@door_state = DataItem.new('door_state'))

      # Initialize data items
      @link.value = 'ENABLED'
      @door_state.value = "OPEN"

      @material.value = "'ROUND 440C THING', 3.27, 11.23"

      @system.normal

      @open_chuck_interface = OpenChuck.new(self)
      @close_chuck_interface = CloseChuck.new(self, @open_chuck_interface)

      @open_door_interface = OpenDoor.new(self)
      @close_door_interface = CloseDoor.new(self, @open_door_interface)

      @material_load_interface = MaterialLoad.new(self)
      @material_unload_interface = MaterialUnload.new(self)

      @cycle_time = 1
      @has_material = false

      load_time_limit(5 * 60)
      unload_time_limit(5 * 60)

      load_failed_time_limit(30)
      unload_failed_time_limit(30)

      create_statemachine
    end

    def start
      @adapter.start
    end
  
    def stop
      @adapter.stop
    end

    def load_time_limit(limit)
      @material_load_interface.processing_time_limit = limit
    end

    def load_failed_time_limit(limit)
      @material_load_interface.fail_time_limit = limit
    end

    def unload_time_limit(limit)
      @material_unload_interface.processing_time_limit = limit
    end

    def unload_failed_time_limit(limit)
      @material_unload_interface.fail_time_limit = limit
    end

    def event(source, name, value, code = nil, text = nil)
      puts "CNC Received #{name} #{value} from #{source}"
      action_name = value.downcase
      if action_name == 'fail'
        action = :failure
      else
        action = action_name.to_sym
      end

      case name
      when "OpenChuck"
        @open_chuck_interface.statemachine.send(action)

      when "CloseChuck"
        @close_chuck_interface.statemachine.send(action)

      when "ChuckState"
        action = "chuck_#{action}".to_sym
        @cnc_chuck_state = value

        @close_chuck_interface.statemachine.send(action)
        @open_chuck_interface.statemachine.send(action)

      when "OpenDoor"
        @open_door_interface.statemachine.send(action)

      when "CloseDoor"
        @close_door_interface.statemachine.send(action)

      when 'MaterialLoad'
        @material_load_interface.statemachine.send(action)

      when 'MaterialUnload'
        @material_unload_interface.statemachine.send(action)
      end

      super(source, name, value, code, text)

    rescue
      puts $!, $!.backtrace
      raise
    end
    
    def cnc_not_ready
      @adapter.gather do
        @open_chuck_interface.deactivate
        @close_chuck_interface.deactivate
        @open_door_interface.deactivate
        @close_door_interface.deactivate
        @material_load_interface.deactivate
        @material_unload_interface.deactivate
      end
      true
    end

    def activate
      if @faults.empty? and @cnc_controller_mode == 'AUTOMATIC' and
          @link.value == 'ENABLED' and @robot_controller_mode == 'AUTOMATIC' and
          @robot_execution == 'ACTIVE' and @system.normal? and @robot_availability == 'AVAILABLE'
        puts "Becomming operational"
        @statemachine.make_operational
      else
        puts "Still not ready"
        puts "  There are robot faults: #{@faults.inspect}" unless @faults.empty?
        puts "  Mode is not AUTOMATIC: #{@cnc_controller_mode}" unless @cnc_controller_mode == 'AUTOMATIC'
        puts "  Link is not ENABLED: #{@link.value}" unless @link.value == 'ENABLED'
        puts "  System condition is not normal" unless @system.normal?
        puts "  Robot is not active" unless @robot_execution == 'ACTIVE'
        puts "  Robot is not available: #@robot_availability" unless @robot_availability == 'AVAILABLE'
        @statemachine.still_not_ready
      end
      true
    end

    def enable
      @adapter.gather do
        @link.value = 'ENABLED'
      end
      true
    end

    def disable
      @adapter.gather do
        @link.value = 'DISABLED'
      end
    end

    def reset_cnc
      puts "*** Resetting Controller ***"
      @adapter.gather do
        @system.normal
      end
      reset_history
      true
    end

    def cycling
      # Check preconditions for a cycle start. Chuck has to be closed and
      # door must be shut.
      if @door_state.value != 'CLOSED' or @cnc_chuck_state != 'CLOSED'
        puts "*** Door #{@door_state.value} or Chuck #{@cnc_chuck_state} is not correct, failing"
        @adapter.gather do
          @system.add('FAULT', 'Door or Chuck in invalid state', 'CYCLE')
        end
        @statemachine.fault
      end
      true
    end

    def loading
      @material_unload_interface.deactivate
      @material_load_interface.activate
      true
    end

    def unloading
      @material_load_interface.deactivate
      @material_unload_interface.activate
      true
    end

    def exit_loading
      @material_load_interface.deactivate
    end

    def exit_unloading
      @material_unload_interface.deactivate
    end

    def operational
      puts "****** Beginning New Part ******"
      @open_chuck_interface.activate
      @close_chuck_interface.activate
      @open_door_interface.activate
      @close_door_interface.activate

      if @has_material
        @statemachine.unloading
      else
        @statemachine.loading
      end
      true
    end

    def exiting_idle
      if @has_material
        @statemachine.unloading
      else
        @statemachine.loading
      end
      true
    end

    def loaded
      @has_material = true
      true
    end

    def unloaded
      @has_material = false
      true
    end

    def idle
      if @has_material
        @material_load_interface.deactivate
        @material_unload_interface.idle
      else
        @material_unload_interface.deactivate
        @material_load_interface.idle
      end
    end

    def reset_history
      @statemachine.get_state(:operational).reset
      true
    end
  
    def add_conditions
    end

    def completed(child)
      puts "** Received completed from #{child.class}"
      @statemachine.complete
    end

    def failed(child)
      puts "** Received failed from #{child.class}"
      @statemachine.failed
    end

    def create_statemachine
      ctx = self
      sm = Statemachine.build do
        startstate :disabled

        superstate :base do
          # Condition handling
          event :robot_system_fault, :fault, :reset_history
          event :robot_system_normal, :activated
          event :robot_system_warning, :activated

          # From the robot
          event :robot_availability_unavailable, :activated
          event :robot_controller_mode_automatic, :activated
          event :robot_execution_active, :activated

          # user commands
          event :cnc_controller_mode_automatic, :activated
          event :cnc_controller_mode_manual, :activated
          event :disable, :activated, :disable
          event :enable, :activated, :enable
          event :reset_cnc, :activated, :reset_cnc


          superstate :disabled do
            default :not_ready
            default_history :not_ready
            on_entry :cnc_not_ready

            # Some robot transitions to get us going again
            event :robot_material_load_ready, :activated
            event :robot_material_unload_ready, :activated


            # Ways to transition out of not ready state...
            state :not_ready do
              on_entry :cnc_not_ready
              default :not_ready
            end

            state :fault do
              on_entry :cnc_not_ready
              default :fault
            end
          end

          state :activated do
            on_entry :activate
            event :make_operational, :operational_H
            event :still_not_ready, :disabled_H
          end

          superstate :operational do
            on_entry :operational
            default_history :loading

            event :loading, :loading
            event :unloading, :unloading

            event :robot_controller_mode_manual, :activated, :reset_history
            event :robot_controller_mode_manual_data_input, :activated, :reset_history
            event :robot_execution_ready, :activated, :reset_history
            event :robot_execution_stopped, :activated, :reset_history

            state :idle do
              on_entry :idle

              event :robot_material_load_ready, :idle, :exiting_idle
              event :robot_material_unload_ready, :idle, :exiting_idle
            end

            state :cycle_start do
              on_entry :cycling
              default :cycle_start

              event :cnc_execution_ready, :unloading
              event :fault, :fault
            end

            state :loading do
              on_entry :loading
              on_exit :exit_loading
              default :loading

              event :failed, :idle
              event :complete, :cycle_start, :loaded
            end

            state :unloading do
              on_entry :unloading
              on_exit :exit_unloading
              default :unloading

              event :failed, :idle
              event :complete, :loading, :unloaded
            end
          end
        end

        context ctx
      end
      sm.tracer = STDOUT
      sm.name = 'CNC'
    end
  end
end

if $0 == __FILE__
  load "graph_generator.rb"
end
