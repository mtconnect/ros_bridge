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
    Given robot Availability is Available
    And robot ControllerMode is Automatic
    And robot Execution is Active
    And robot MaterialLoad is Ready
    And robot MaterialUnload is Ready
    And robot OpenDoor is Ready
    And robot CloseDoor is Ready
    And robot OpenChuck is Ready
    And robot CloseChuck is Ready
    And cnc ControllerMode is Automatic
    Then cnc MaterialLoad should be Active
    And cnc MaterialUnload should be Not_Ready
  }
end


Given(/^Chuck is closed$/) do
  steps %Q{
    Given robot CloseChuck becomes Active
    And cnc ChuckState becomes Closed
    And robot CloseChuck becomes Ready
    Then cnc ChuckState should be Closed
  }
end

Given(/^Door is closed$/) do
  steps %Q{
    Given robot CloseDoor becomes Active
    Then after 1.2 seconds cnc CloseDoor should be Complete
    And robot CloseDoor becomes Ready
    Then cnc DoorState should be Closed
  }
end

Given(/^(cnc|robot) ([A-Za-z]+) (?>is|becomes) ([A-Za-z_]+)$/) do |target, item, value|
  cnc.event(target, item, value.upcase)
end

When(/^(robot|cnc) faults ([A-Za-z_]+) with "(.*?)"$/) do |target, item, message|
  cnc.event(target, item, 'Fault', item, message)
end

When(/^(robot|cnc) clears ([A-Za-z_]+)$/) do |target, item|
  cnc.event(target, item, 'Normal', item)
end

Then(/^(machine|(?>open|close) (?>door|chuck)|material (?>load|unload)) state should be ([a-z_]+)$/) do |machine, state|
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
