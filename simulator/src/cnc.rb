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
    attr_accessor :robot_controller_mode, :robot_material_load, :robot_material_unload,
                  :robot_open_chuck, :robot_close_chuck, :robot_open_door, :robot_close_door,
                  :robot_execution, :robot_availability

    attr_accessor :cnc_controller_mode, :cnc_execution, :cnc_availability, :cnc_chuck_state

    attr_reader :adapter, :open_chuck, :close_chuck, :door_state, :open_door, :close_door,
                :material_load, :material_unload, :link, :system, :has_material,
                :material_load_interface, :material_unload_interface,
                :open_chuck_interface, :close_chuck_interface,
                :open_door_interface, :close_door_interface

    attr_accessor :has_material


    def initialize(control, port = 7879)
      super(port)

      # Initilize robot instance variables
      @robot_controller_mode = @robot_material_load = @robot_material_unload = @robot_open_chuck =
        @robot_close_chuck = @robot_open_door = @robot_close_door =
            @robot_execution = @robot_availability = nil
      @cnc_chuck_state = @cnc_controller_mode = @cnc_execution = @cnc_availability = nil

      @control = control

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

      if $simulation
        @adapter.data_items << (@exec = DataItem.new('execution'))
        @adapter.data_items << (@avail = DataItem.new('avail'))
        @adapter.data_items << (@mode = DataItem.new('mode'))

        @avail.value = 'AVAILABLE'
        @exec.value = 'READY'
        @mode.value = 'AUTOMATIC'
      end

      # Initialize data items
      @link.value = 'ENABLED'
      @door_state.value = "OPEN"

      @material.value = "'ROUND 440C THING', 3.27, 11.23"

      @system.normal

      @open_chuck_interface = OpenChuck.new(self, @control)
      @close_chuck_interface = CloseChuck.new(self, @control, @open_chuck_interface)
      if $simulation
        @open_chuck_interface.simulate = true
        @close_chuck_interface.simulate = true
      end

      @open_door_interface = OpenDoor.new(self)
      @close_door_interface = CloseDoor.new(self, @open_door_interface)

      @material_load_interface = MaterialLoad.new(self)
      @material_unload_interface = MaterialUnload.new(self)

      @has_material = false
      @fail_next = false

      @events = []

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

    def status
      return <<EOT
Cnc state  = #{@statemachine.state}
Load Material state = #{@material_load_interface.state}
Unload Material state = #{@material_unload_interface.state}
Open Chuck state = #{@open_chuck_interface.statemachine.state}
Close Chuck state = #{@close_chuck_interface.statemachine.state}
Open Door state = #{@open_door_interface.statemachine.state}
Close Door state = #{@close_door_interface.statemachine.state}
EOT
    end

    def events(count)
      @events.last(count).join("\n")
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
      @events.push "#{source}: #{name} #{value} '#{code}' '#{text}'"
      @events.shift if @events.length > 500

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
        puts "  Robot is not available: #{@robot_availability}" unless @robot_availability == 'AVAILABLE'
        puts "  Robot controller mode is not automatic: #{@robot_controller_mode}" unless @robot_controller_mode == "AUTOMATIC"
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
      @control.puts "* reset" unless $simulation

      @close_chuck_interface.reset
      @open_chuck_interface.reset
      @open_door_interface.reset
      @close_door_interface.reset
      @material_load_interface.reset
      @material_unload_interface.reset

      true
    end

    def cycling
      # Check preconditions for a cycle start. Chuck has to be closed and
      # door must be shut.
      if @fail_next
        @adapter.gather do
          @system.add('FAULT', 'Cycle failed to start', 'CYCLE')
        end
        @statemachine.fault
        @fail_next = false
      elsif @door_state.value != 'CLOSED' or @cnc_chuck_state != 'CLOSED'
        puts "*** Door #{@door_state.value} or Chuck #{@cnc_chuck_state} is not correct, failing"
        @adapter.gather do
          @system.add('FAULT', 'Door or Chuck in invalid state', 'CYCLE')
        end
        @statemachine.fault
      else
        unless $simulation
          @control.puts "* start"
        else
          @adapter.gather do
            @exec.value = 'ACTIVE'
          end
          Thread.new do
            sleep 10
            @adapter.gather do
              @exec.value = 'READY'
            end
          end
        end
      end
      true
    end

    def loading
      unless @has_material
        @material_unload_interface.deactivate
        @material_load_interface.activate
      end
      true
    end

    def unloading
      if @has_material
        @material_load_interface.deactivate
        @material_unload_interface.activate
      end
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

      @control.puts "* reset" unless $simulation

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
      case child
      when Request
        @statemachine.complete
      end
    end

    def failed(child)
      puts "** Received failed from #{child.class}"
      case child
      when Request
        @statemachine.failed

      when Response
        @adapter.gather do
          @system.add('FAULT', "#{child.class.name} Failed", child.class.name)
        end
        @statemachine.fault
      end
    end

    def fail_next(name, value)
      if name == 'exec'
        @fail_next = true
      else
        inst = "@#{name}_interface".to_sym
        if (var = instance_variable_get(inst)) and var.respond_to?(:'fail_next=')
          var.fail_next = value
        else
          raise "Cannot find variable with name #{var}"
        end
      end
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
          event :robot_availability_available, :activated
          event :robot_controller_mode_automatic, :activated
          event :robot_controller_mode_manual, :activated
          event :robot_controller_mode_manual_data_input, :activated
          event :robot_execution_active, :activated
          event :robot_execution_ready, :activated
          event :robot_execution_stopped, :activated
          event :robot_execution_interupted, :activated

          # CNC controller feedback
          event :cnc_controller_mode_automatic, :activated
          event :cnc_controller_mode_manual, :activated
          event :cnc_controller_mode_manual_data_input, :activated

          # User commands
          event :disable, :activated, :disable
          event :enable, :activated, :enable
          event :reset_cnc, :activated, :reset_cnc

          # failure
          event :fault, :fault

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
              default :idle

              event :robot_material_load_ready, :idle, :exiting_idle
              event :robot_material_unload_ready, :idle, :exiting_idle
            end

            state :cycle_start do
              on_entry :cycling
              default :cycle_start

              event :cnc_execution_ready, :unloading
              event :fault, :fault
              event :cnc_fault, :fault
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
