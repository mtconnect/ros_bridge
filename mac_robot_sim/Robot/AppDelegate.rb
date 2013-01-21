#
#  AppDelegate.rb
#  Robot
#
#  Created by William Sobel on 12/17/12.
#  Copyright 2012 William Sobel. All rights reserved.
#

require 'streamer'
require 'robot'

class AppDelegate
  attr_accessor :window, :adapter_port, :agent_url
  
  # Robot
  attr_accessor :controller_mode, :execution
  attr_accessor :system, :material_load, :material_unload
  attr_accessor :open_door, :close_door, :open_chuck, :close_chuck
  attr_accessor :ready, :fail
  
  # Cnc variables
  attr_accessor :cnc_controller_mode, :cnc_execution, :cnc_system
  attr_accessor :cnc_material_load, :cnc_material_unload
  attr_accessor :cnc_open_chuck, :cnc_close_chuck, :chuck_meter
  attr_accessor :cnc_open_door, :cnc_close_door, :door_meter
  attr_accessor :cnc_door_state, :cnc_chuck_state, :cnc_link_state
  attr_accessor :cnc_system_text
  
  def applicationDidFinishLaunching(a_notification)
    # Insert code here to initialize your application
    @controller_mode.selectItemAtIndex(1)
    @execution.selectItemAtIndex(0)
    @streamer = nil
  end
  
  def update_state_meter(name, value)
    level = case value
    when 'OPEN'
      1
    when 'UNLATCHED'
      2
    when 'CLOSED'
      3
    else
      0
    end
    meter = if name == 'DoorState'
      @door_meter
    elsif name == 'ChuckState'
      @chuck_meter
    end
    meter.setIntValue(level) if meter
  end

  def update_cell_background(obj, value)
    case value
    when 'ACTIVE', 'COMPLETE'
      obj.setBackgroundColor(NSColor.greenColor)
    when 'FAIL'
      obj.setBackgroundColor(NSColor.redColor)
    else
      obj.setBackgroundColor(NSColor.whiteColor)
    end
  end

  def update_adapter(sender)
    return unless @robot
    
    if @ready.state == NSOnState
      @robot.ready
    else
      @robot.not_ready
    end
    @robot.gather do
      @robot.controller_mode.value = @controller_mode.stringValue
      @robot.execution.value = @execution.stringValue
      if @system.state == NSOnState
        @robot.system.add('fault', 'System fault', '1')
      else
        @robot.system.normal
      end
    end
  end

  def update_from_robot
    @material_load.setStringValue(v = @robot.material_load.value)
    update_cell_background(@material_load, v)
    
    @material_unload.setStringValue(v = @robot.material_unload.value)
    update_cell_background(@material_unload, v)
    
    @close_chuck.setStringValue(v = @robot.close_chuck.value)
    update_cell_background(@close_chuck, v)
    
    @open_chuck.setStringValue(v = @robot.open_chuck.value)
    update_cell_background(@open_chuck, v)
    
    @close_door.setStringValue(v = @robot.close_door.value)
    update_cell_background(@close_door, v)
    
    @open_door.setStringValue(v = @robot.open_door.value)
    update_cell_background(@open_door, v)
  end

  def load_material(sender)
    return unless @robot

    Thread.new { @robot.load_material }
  end

  def unload_material(sender)
    return unless @robot

    Thread.new { @robot.unload_material }
  end

  def close_the_chuck(sender)
    return unless @robot
    
    Thread.new { @robot.close_the_chuck }
  end

  def open_the_chuck(sender)
    return unless @robot
    
    Thread.new { @robot.open_the_chuck }
  end


  def close_the_door(sender)
    return unless @robot
    
    Thread.new { @robot.close_the_door }
  end

  def open_the_door(sender)
    return unless @robot
    
    Thread.new { @robot.open_the_door }
  end


  def start_adapter(sender)
    @robot = Robot.new(@adapter_port.intValue, self)
    @robot.start
    update_adapter(sender)
    
    @streamer = MTConnect::Streamer.new(@agent_url.stringValue)
    @streamer.start do |name, value, code, text|
      case value
      when 'Unavailable', 'Fault', 'Warning', 'Normal'
        @cnc_system.setStringValue(value)
        @cnc_system_text.setStringValue(text.to_s)
      else
        variable = name.split(/([A-Z][a-z]+)/).delete_if(&:empty?).map(&:downcase).join('_')
        puts "#{variable}: #{value}"
        obj = instance_variable_get("@cnc_#{variable}")
        if obj
          obj.setStringValue(value)
          if name =~ /State$/
            update_state_meter(name, value)
          elsif name =~ /Material|Execution|Open|Close/
            update_cell_background(obj, value)
          end
        end
      end
    end
  end
  
  def stop_adapter(sender)
    @streamer.stop
    @robot.stop
  end
end

