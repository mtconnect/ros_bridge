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

module Cnc
  class CncContext < MTConnect::Context  
    include MTConnect
    attr_accessor :robot_controller_mode, :robot_material_load, :robot_material_unload, :robot_controller_mode,
                  :robot_open_chuck, :robot_close_chuck, :robot_open_door, :robot_close_door,
                  :robot_execution, :robot_availability

    attr_accessor :cnc_controller_mode, :cnc_execution, :cnc_availability, :cnc_chuck_state

    attr_reader :adapter, :open_chuck, :close_chuck, :door_state, :open_door, :close_door,
                :material_load, :material_unload, :link

    attr_accessor :cycle_time, :unload_failed_time_limit, :load_time_limit, :unload_time_limit,
                  :load_failed_time_limit


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
      # @adapter.data_items << (@chuck_state = DataItem.new('chuck_state'))
      # @adapter.data_items << (@exec = DataItem.new('exec'))
      # @adapter.data_items << (@mode = DataItem.new('mode'))
      @adapter.data_items << (@system = SimpleCondition.new('infc_system'))

      # Controller does not have a door signal, so we need to simulate it here.
      @adapter.data_items << (@door_state = DataItem.new('door_state'))

      @interfaces = [@material_load, @material_unload]

      # Initialize data items
      @link.value = 'ENABLED'
      @door_state.value = "OPEN"

      @material.value = "'ROUND 440C THING', 3.27, 11.23"

      @system.normal

      @open_chuck_interface = OpenChuck.new(self)
      @close_chuck_interface = CloseChuck.new(self, @open_chuck_interface)

      @open_door_interface = OpenDoor.new(self)
      @close_door_interface = CloseDoor.new(self, @open_door_interface)

      @cycle_time = 1

      @load_timer = @unload_timer = nil
      @load_time_limit = @unload_time_limit = 5 * 60

      @load_failed_timer = @unload_failed_timer = nil
      @load_failed_time_limit = @unload_failed_time_limit = 30

      create_statemachine
    end

    def start
      @adapter.start
    end
  
    def stop
      @adapter.stop
    end

    def event(source, name, value, code = nil, text = nil)
      puts "CNC Received #{name} #{value} from #{source}"
      case name
      when "OpenChuck"
        action = value.downcase.to_sym
        @open_chuck_interface.statemachine.send(action)

      when "CloseChuck"
        action = value.downcase.to_sym
        @close_chuck_interface.statemachine.send(action)

      when "ChuckState"
        action = "chuck_#{value.downcase}".to_sym
        @cnc_chuck_state = value

        @close_chuck_interface.statemachine.send(action)
        @open_chuck_interface.statemachine.send(action)

      when "OpenDoor"
        action = value.downcase.to_sym
        @open_door_interface.statemachine.send(action)

      when "CloseDoor"
        action = value.downcase.to_sym
        @close_door_interface.statemachine.send(action)

      else
        super(source, name, value, code, text)
      end
    end
    
    def cnc_not_ready
      @adapter.gather do
        @open_chuck_interface.deactivate
        @close_chuck_interface.deactivate
        @open_door_interface.deactivate
        @close_door_interface.deactivate
        @interfaces.each { |i| i.value = 'NOT_READY' }
      end
    end

    def cnc_ready
      puts "****** Beginning New Part ******"

      @adapter.gather do
        @interfaces.each { |i| i.value = 'READY' }
      end
      @open_chuck_interface.activate
      @close_chuck_interface.activate
      @open_door_interface.activate
      @close_door_interface.activate
    end

    def activate
      if @faults.empty? and @cnc_controller_mode == 'AUTOMATIC' and
          @link.value == 'ENABLED' and @robot_controller_mode == 'AUTOMATIC' and
          @robot_material_load == 'READY' and @robot_material_unload == 'READY' and
          @robot_execution == 'ACTIVE' and @system.normal? and @robot_availability == 'AVAILABLE'
        puts "Becomming operational"
        @statemachine.make_operational
      else
        puts "Still not ready"
        puts "  There are robot faults: #{@faults.inspect}" unless @faults.empty?
        puts "  Mode is not AUTOMATIC: #{@cnc_controller_mode}" unless @cnc_controller_mode == 'AUTOMATIC'
        puts "  Robot Material Load is not READY: #@robot_material_load" unless @robot_material_load == 'READY'
        puts "  Link is not ENABLED: #{@link.value}" unless @link.value == 'ENABLED'
        puts "  Robot material unload is not READY: #@robot_material_unload" unless @robot_material_unload == 'READY'
        puts "  System condition is not normal" unless @system.normal?
        puts "  Robot is not active" unless @robot_execution == 'ACTIVE'
        puts "  Robot is not available: #@robot_availability" unless @robot_availability == 'AVAILABLE'
        @statemachine.still_not_ready
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

    def reset_cnc
      puts "*** Resetting Controller ***"
      @adapter.gather do
        @system.normal
      end
      reset_history
    end

    def cycling
      # Check preconditions for a cycle start. Chuck has to be closed and
      # door must be shut.
      if @door_state.value != 'CLOSED' or @cnc_chuck_state != 'CLOSED'
        @adapter.gather do
          @system.add('FAULT', 'Door or Chuck in invalid state', 'CYCLE')
        end
        @statemachine.fault
      else
        @adapter.gather do
          @material_load.value = 'READY'
        end
      end
    end

    def material_load_active
      @adapter.gather do
        @material_load.value = 'ACTIVE'
      end
    end

    def start_load_active_timer
      @load_timer = Thread.new do
        sleep @load_time_limit
        @statemachine.material_load_timeout
      end
    end

    def stop_load_active_timer
      @load_timer.kill if @load_timer
      @load_timer = nil
    end


    def start_unload_active_timer
      @unload_timer = Thread.new do
        sleep @unload_time_limit
        @statemachine.material_unload_timeout
      end
    end

    def stop_unload_active_timer
      @unload_timer.kill if @unload_timer
      @unload_timer = nil
    end

    def material_load_ready
      @adapter.gather do
        @material_load.value = 'READY'
      end
    end

    def load_failed
      @adapter.gather do
        @material_load.value = 'FAIL'
      end
      @load_failed_timer = Thread.new do
        sleep @load_failed_time_limit
        @statemachine.material_load_failed_timeout
      end
    end

    def stop_load_failed_timer
      @load_failed_timer.kill if @load_failed_timer
      @load_failed_timer = nil
    end

    def material_unload_active
      @adapter.gather do
        @material_unload.value = 'ACTIVE'
      end
    end

    def material_unload_ready
      @adapter.gather do
        @material_unload.value = 'READY'
      end
    end

    def unload_failed
      @adapter.gather do
        @material_unload.value = 'FAIL'
      end
      @unload_failed_timer = Thread.new do
        sleep @unload_failed_time_limit
        @statemachine.material_unload_failed_timeout
      end
    end
    
    def stop_unload_failed_timer
      @unload_failed_timer.kill if @unload_failed_timer
      @unload_failed_timer = nil
    end

    def reset_history
      @statemachine.get_state(:operational).reset
    end
  
    def add_conditions
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
          event :robot_material_load_ready, :activated
          event :robot_material_load_not_ready, :activated
          event :robot_material_unload_ready, :activated
          event :robot_material_unload_not_ready, :activated

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
            startstate :handling
            default_history :handling
            event :robot_controller_mode_manual, :activated, :reset_history
            event :robot_controller_mode_manual_data_input, :activated, :reset_history
            event :robot_execution_ready, :activated, :reset_history
            event :robot_execution_stopped, :activated, :reset_history

            state :cycle_start do
              on_entry :cycling
              event :cnc_execution_ready, :material_unload
              event :fault, :fault
            end

            superstate :handling do
              on_entry :cnc_ready
              default_history :material_load
              event :robot_material_load_ready, :activated

              state :material_load do
                on_entry :material_load_active
                default :material_load

                event :robot_material_load_active, :material_load, :start_load_active_timer
                event :robot_material_load_complete, :cycle_start, :stop_load_active_timer
                event :robot_material_load_fail, :material_load_failed
                event :robot_material_load_not_ready, :activated, :reset_history
                event :robot_material_unload_ready, :material_load
                event :robot_material_unload_not_ready, :material_load
                event :material_load_timeout, :material_load_failed
              end

              state :material_load_failed do
                on_entry :load_failed
                default :material_load_failed

                event :robot_material_load_fail, :material_load_failed
                event :robot_material_load_ready, :material_load, :stop_load_failed_timer
                event :material_load_failed_timeout, :disabled
              end

              state :material_unload do
                on_entry :material_unload_active
                default :material_unload

                event :robot_material_unload_active, :material_unload, :start_unload_active_timer
                event :robot_material_unload_complete, :handling, :stop_unload_active_timer
                event :robot_material_unload_not_ready, :activated, :reset_history
                event :robot_material_unload_fail, :material_unload_failed
                event :robot_material_unload_ready, :material_unload
                event :robot_material_load_not_ready, :material_unload
                event :material_unload_timeout, :material_unload_failed
              end

              state :material_unload_failed do
                on_entry :unload_failed
                default :material_unload_failed

                event :robot_material_unload_fail, :material_unload_failed
                event :robot_material_unload_ready, :material_unload, :stop_unload_failed_timer
                event :material_unload_failed_timeout, :disabled
              end
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
