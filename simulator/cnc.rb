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
    attr_accessor :controller_mode
  
    def initialize(port = 7879)
      super(port)
      
      @adapter.data_items << (@availability_di = DataItem.new('avail'))
      @adapter.data_items << (@load_material_di = DataItem.new('load_mat'))
      @adapter.data_items << (@change_material_di = DataItem.new('change_mat'))
      @adapter.data_items << (@chuck_state_di = DataItem.new('chuck_state'))
      @adapter.data_items << (@link_di = DataItem.new('bf_link'))
      @adapter.data_items << (@top_cut_di = DataItem.new('top_cut'))
      @adapter.data_items << (@system_di = Condition.new('system'))
      @adapter.data_items << (@exec_di = DataItem.new('exec'))
      @adapter.data_items << (@mode_di = DataItem.new('mode'))

      # Initialize data items
      @availability_di.value = "AVAILABLE"
      @load_material_di.value = 'NOT_READY'
      @change_material_di.value = 'NOT_READY'
      @top_cut_di.value = 'NOT_READY'
      @chuck_state_di.value = 'UNLATCHED'
      @link_di.value = 'DISABLED'
      @exec_di.value = 'READY'
      @mode_di.value = 'MANUAL'
      
      @change_ready = false
      @load_ready = false
      
      @system_di.normal
      
      @adapter.start    
    end
  
    def stop
      @adapter.stop
    end
    
    def cnc_not_ready
      @adapter.gather do
        @exec_di.value = 'READY'
        @top_cut_di.value = 'NOT_READY'
        @load_material_di.value = 'NOT_READY'
        @change_material_di.value = 'NOT_READY'        
        @chuck_state_di.value = 'UNLATCHED'
      end
    end
    
    def activate
      if @faults.empty? and @mode_di.value == 'AUTOMATIC' and 
        @link_di.value == 'ENABLED'
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

    def cnc_ready
      @adapter.gather do
        @exec_di.value = 'READY'
        @top_cut_di.value = 'READY'
        @load_material_di.value = 'READY'
        @change_material_di.value = 'READY'        
        @chuck_state_di.value = 'OPEN'
      end
    end
    
    def cycle_start
      @adapter.gather do
        @exec_di.value = 'ACTIVE'
        @change_material_di.value = 'READY'
        @load_material_di.value = 'READY'
      end
      
      if @load_ready
        @statemachine.load_material
      elsif @change_ready
        @statemachine.change_material
      else
        @statemachine.disabled
      end
    end
    
    def change_not_ready
      @change_ready = false
    end
    
    def change_ready
      @change_ready = true
    end
    
    def load_not_ready
      @load_ready = false
    end
    
    def load_ready
      @load_ready = true
    end
    
    def change_material_ready
      @adapter.gather do
        @change_material_di.value = 'READY'
      end
    end
    
    def change_material
      @adapter.gather do
        @change_material_di.value = 'ACTIVE'
      end
    end

    def load_material
      @adapter.gather do
        @load_material_di.value = 'ACTIVE'
      end
    end

    def load_material_ready
      @adapter.gather do
        @load_material_di.value = 'READY'
      end
    end
    
    def begin_top_cut
      @adapter.gather do 
        @top_cut_di.value = 'ACTIVE'
        add_conditions
      end
      Thread.new do
        sleep 1
        @statemachine.top_cut_completed
      end
    end
    
    def top_cut_completed
      @adapter.gather do 
        @top_cut_di.value = 'COMPLETE'
        add_conditions
      end
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
      event :load_material_not_ready, :activated, :load_not_ready
      event :load_material_ready, :activated, :load_ready
      event :change_material_not_ready, :activated, :change_not_ready
      event :change_material_ready, :activated, :change_ready
      event :fault, :fault, :reset_history
      event :automatic, :activated, :automatic_mode
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
      
        state :bar_feeder_empty do
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
        
        state :ready do
          default :ready
          on_entry :cnc_ready
          event :cycle_start, :cycle_start
        end
        
        state :cycle_start do
          on_entry :cycle_start
          event :load_material, :load_material
          event :change_material, :change_material
        end
        
        state :change_material do
          on_entry :change_material
          event :change_material_active, :change_material
          event :change_material_complete, :ready
        end
        
        state :load_material do
          on_entry :load_material
          event :load_material_active, :load_material
          event :load_material_complete, :one_cycle
          event :load_material_not_ready, :load_material, :load_not_ready
        end
        
        state :one_cycle do
          on_entry :one_cycle
          event :load_material_not_ready, :one_cycle, :load_not_ready
          event :normal, :one_cycle
          event :cycle_completed, :cycle_start
        end
                
        [:top_cut].each do |interface|
          active = "#{interface}_active".to_sym
          fail = "#{interface}_fail".to_sym
          failed = "#{interface}_failed".to_sym
          complete = "#{interface}_complete".to_sym
          completed = "#{interface}_completed".to_sym
          ready = "#{interface}_ready".to_sym
          cnc_fail = "cnc_#{interface}_fail".to_sym
          
          trans :ready, active, interface
            
          # Active state of interface  
          state interface do
            on_entry "begin_#{interface}".to_sym
            event ready, fail
            event completed, complete
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
            event ready, :ready
          end
        end
        
        trans :top_cut_complete, :top_cut_ready, :load_material      
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
  Cnc.cnc.to_dot(:output => 'graph')
end