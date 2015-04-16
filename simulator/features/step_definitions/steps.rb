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

Given(/^Devices are in initial state$/) do
  steps %Q{
    Given robot Device Availability is Available
    And robot Controller ControllerMode is Automatic
    And robot Controller Execution is Active
    And robot MaterialHandlerInterface MaterialLoad is Ready
    And robot MaterialHandlerInterface MaterialUnload is Ready
    And robot DoorInterface Open is Ready
    And robot DoorInterface Close is Ready
    And robot ChuckInterface Open is Ready
    And robot ChuckInterface Close is Ready
    And cnc Controller ControllerMode is Automatic
    Then cnc MaterialLoad should be Active
    And cnc MaterialUnload should be Not_Ready
  }
end


Given(/^Chuck is closed$/) do
  steps %Q{
    Given robot ChuckInterface Close becomes Active
    Then after 1.2 seconds cnc CloseChuck should be Complete
    And robot ChuckInterface Close becomes Ready
    Then cnc ChuckState should be Closed
  }
end

Given(/^Door is closed$/) do
  steps %Q{
    Given robot DoorInterface Close becomes Active
    Then after 1.2 seconds cnc CloseDoor should be Complete
    And robot DoorInterface Close becomes Ready
    Then cnc DoorState should be Closed
  }
end

Given(/^(cnc|robot) ([A-Za-z]+) ([A-Za-z]+) (?>is|becomes) ([A-Za-z_]+)$/) do |target, comp, item, value|
  cnc.event(target, comp, item, value.upcase)
end

When(/^(robot|cnc) faults ([A-Za-z]+) ([A-Za-z_]+) with "(.*?)"$/) do |target, comp, item, message|
  cnc.event(target, comp, item, 'Fault', item, message)
end

When(/^(robot|cnc) clears ([A-Za-z]+) ([A-Za-z_]+)$/) do |target, comp, item|
  cnc.event(target, comp, item, 'Normal', item)
end

Then(/^(machine|(?>open|close) (?>door|chuck)|material (?>load|unload)) state should be ([a-zA-Z_]+)$/) do |machine, state|
  sm = machine_for(machine)
  sm.state.should == state.to_sym
end

Then(/^(?>after ([0-9.]+) second(?>s)? )?(cnc|robot) ([A-Za-z]+) should be ([A-Za-z_]+)$/) do |delay, source, item, value|
  sleep delay.to_f if delay
  value_for(source, item).should == value.upcase
end

Then(/^cnc should have fault$/) do
  cond = cnc.system
  cond.should_not be_normal
end

When(/^cnc is reset$/) do
  cnc.statemachine.reset_cnc
end

When(/^faults are cleared$/) do
  @cnc.adapter.gather do
    cnc.system.normal
  end
end

Then(/^cnc fault should have code "(.*?)"$/) do |code|
  cond = cnc.system
  cond.active.first.should_not be_nil
  cond.active.first.code.should == code
end

Given(/^simulate fail ([A-Za-z_]+)$/) do |name|
  cnc.fail_next(name, true)
end
