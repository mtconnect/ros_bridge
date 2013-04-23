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

$: << File.dirname(__FILE__) + '/../../src'

require 'cnc'
require 'rspec/mocks'

begin require 'rspec/expectations'; rescue LoadError; require 'spec/expectations'; end

module CncHelper
  def cnc
    control = Object.new
    class <<control
      def puts(*args); end
    end
    @cnc = Cnc::CncContext.new(control) unless @cnc
    @cnc
  end

  def value_for(source, item)
    element = item.split(/([A-Z][a-z]+)/).delete_if(&:empty?).map(&:downcase).join('_')
    if source == 'robot'
      method = "#{source}_#{element}".to_sym
    else
      method = element.to_sym
      method = "cnc_#{method}".to_sym unless cnc.respond_to? method
    end
    value = cnc.send(method)
    value = value.value if MTConnect::DataItem === value
    value
  end

  def machine_for(name)
    case name
    when "machine"
      cnc.statemachine

    when "open door"
      cnc.open_door_interface.statemachine

    when "close door"
      cnc.close_door_interface.statemachine

    when "open chuck"
      cnc.open_chuck_interface.statemachine

    when "close chuck"
      cnc.close_chuck_interface.statemachine

    when "material load"
      cnc.material_load_interface.statemachine

    when "material unload"
      cnc.material_unload_interface.statemachine
    end
  end
end

World(CncHelper)
