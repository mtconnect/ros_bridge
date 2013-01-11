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

require 'adapter'
require 'data_item'

include MTConnect

class Robot
  attr_reader :availability, :controller_mode, :execution, :material_load, :system
  attr_reader :material_unload, :open_chuck, :close_chuck
  attr_reader :open_door, :close_door, :adapter
  
  def initialize(port, cnc)
    @adapter = Adapter.new(port)
    @cnc = cnc

    @adapter.data_items << (@availability = DataItem.new('avail'))
    @adapter.data_items << (@system = SimpleCondition.new('system'))

    @adapter.data_items << (@controller_mode = DataItem.new('mode'))
    @adapter.data_items << (@execution = DataItem.new('exec'))


    @adapter.data_items << (@material_load = DataItem.new('material_load'))
    @adapter.data_items << (@material_unload = DataItem.new('material_unload'))

    @adapter.data_items << (@open_chuck = DataItem.new('open_chuck'))
    @adapter.data_items << (@close_chuck = DataItem.new('close_chuck'))

    @adapter.data_items << (@open_door = DataItem.new('open_door'))
    @adapter.data_items << (@close_door = DataItem.new('close_door'))
    
    @failed = false
  end

  def start
    @system.normal
    @adapter.start
    not_ready
  end
  
  def stop
    @adapter.stop
  end
  
  def gather(&block)
    @adapter.gather(&block)
  end
  
  def fail(value)
    @failed = value
  end
  
  def ready
    @adapter.gather do
      [@material_load, @material_unload, @open_chuck, @close_chuck,
      @open_door, @close_door].each { |di| di.value = "READY" }
    end
    @cnc.update_from_robot
  end
  
  def not_ready
    @adapter.gather do
      [@material_load, @material_unload, @open_chuck, @close_chuck,
      @open_door, @close_door].each { |di| di.value = "NOT_READY" }
    end
    @cnc.update_from_robot
  end
  
  def stop?
    @failed or !@system.normal?
  end
  
  def update_data_item(item, value)
    @adapter.gather do
      item.value = value
    end
    @cnc.update_from_robot    
  end
  
  def open_the_door
    update_data_item(@open_door, 'ACTIVE')
    sleep 1.5
    update_data_item(@open_door, 'READY')
  end
  
  def close_the_door
    update_data_item(@close_door, 'ACTIVE')
    sleep 1.5
    update_data_item(@close_door, 'READY')
  end
  
  def open_the_chuck
    update_data_item(@open_chuck, 'ACTIVE')
    sleep 1.5
    update_data_item(@open_chuck, 'READY')
  end
  
  def close_the_chuck
    update_data_item(@close_chuck, 'ACTIVE')
    sleep 1.5
    update_data_item(@close_chuck, 'READY')
  end
  
  def load_material
    if @cnc.cnc_material_load.stringValue != 'ACTIVE'
      update_data_item(@material_load, 'FAIL')
      sleep 2
      update_data_item(@material_load, 'READY')
      return
    end
    
    update_data_item(@material_load, 'ACTIVE')

    if @cnc.cnc_door_state.stringValue != 'OPEN'
      open_the_door
    end
    
    if @cnc.cnc_chuck_state.stringValue != 'OPEN'
      open_the_chuck
    end
    
    close_the_chuck
    
    sleep 1.5
    
    close_the_door
    
    sleep 2
    
    update_data_item(@material_load, 'COMPLETE')
    
    sleep 0.5
    
    update_data_item(@material_load, 'READY')
  end
  
  def unload_material
    if @cnc.cnc_material_unload.stringValue != 'ACTIVE' or
      @cnc.cnc_chuck_state.stringValue != 'CLOSED'
      update_data_item(@material_unload, 'FAIL')
      sleep 2
      update_data_item(@material_unload, 'READY')
      return
    end
    
    update_data_item(@material_unload, 'ACTIVE')
    
    if @cnc.cnc_door_state.stringValue != 'OPEN'
      open_the_door
    end
    
    sleep 1.5
    
    open_the_chuck
    
    sleep 2
    
    update_data_item(@material_unload, 'COMPLETE')
    
    sleep 0.5

    update_data_item(@material_unload, 'READY')
  end
end