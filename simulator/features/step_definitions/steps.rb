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

Given(/^(cnc|robot) ([A-Za-z]+) (?>is|becomes) ([A-Za-z_]+)$/) do |target, item, value|
  cnc.event(target, item, value.upcase)
end

Then(/^(top|(?>open|close) (?>door|chuck)|(?>load|unload) material) state should be ([a-z_]+)$/) do |machine, state|
  sm = machine_for(machine)
  sm.state.should == state.to_sym
end

Then(/^(?>after ([0-9.]+) second(?>s)? )?(cnc|robot) ([A-Za-z]+) should be ([A-Za-z_]+)$/) do |delay, source, item, value|
  sleep delay.to_f if delay

  value_for(source, item).should == value.upcase
end

