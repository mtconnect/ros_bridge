#
#  AppDelegate.rb
#  Robot
#
#  Created by William Sobel on 12/17/12.
#  Copyright 2012 William Sobel. All rights reserved.
#

require 'streamer'

class AppDelegate
  attr_accessor :window, :adapter_port
  attr_accessor :controller_mode
  
  # Cnc variables
  attr_accessor :cnc_controller_mode, :cnc_execution, :cnc_system
  attr_accessor :cnc_material_load, :cnc_material_unload
  attr_accessor :cnc_open_chuck, :cnc_close_chuck, :chuck_meter
  attr_accessor :cnc_open_door, :cnc_close_door, :door_meter
  attr_accessor :cnc_door_state, :cnc_chuck_state, :cnc_link_state
  
  def applicationDidFinishLaunching(a_notification)
    # Insert code here to initialize your application
    controller_mode.selectItemAtIndex(1)
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
    end
    meter = if name == 'DoorState'
      @door_meter
    elsif name == 'ChuckState'
      @chuck_meter
    end
    meter.setIntValue(level) if meter
  end

  def update_material_cell(obj, value)
    case value
    when 'ACTIVE', 'COMPLETE'
      obj.setBackgroundColor(NSColor.greenColor)
    when 'FAIL'
      obj.setBackgroundColor(NSColor.redColor)
    else
      obj.setBackgroundColor(NSColor.whiteColor)
    end
  end

  def startAdapter(sender)
    @streamer = MTConnect::Streamer.new('http://localhost:5000/cnc')
    @streamer.start do |name, value|
      case name
      when 'Unavailable', 'Fault', 'Warning', 'Normal'
        @cnc_system.setStringValue(name)
      else
        variable = name.split(/([A-Z][a-z]+)/).delete_if(&:empty?).map(&:downcase).join('_')
        puts "#{variable}: #{value}"
        obj = instance_variable_get("@cnc_#{variable}")
        if obj
          obj.setStringValue(value)
          if name =~ /State$/
            update_state_meter(name, value)
          elsif name =~ /Material|Execution|Open|Close/
            update_material_cell(obj, value)
          end
        end
      end
    end
  end
  
  def stopAdapter(sender)
    @streamer.stop
  end
  
  def update_controller_mode(sender)
  end
end

