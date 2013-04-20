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

$: << File.dirname(__FILE__) + '/../src'

module Helpers
  def unloading
    @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
    @cnc.event('robot', 'CloseDoor', 'ACTIVE')
    @cnc.event('robot', 'CloseChuck', 'ACTIVE')
    sleep 1.2
    @cnc.event('cnc', 'ChuckState', 'CLOSED')
    @cnc.door_state.value.should == 'CLOSED'
    @cnc.cnc_chuck_state.should == 'CLOSED'
    @cnc.event('robot', 'CloseDoor', 'READY')
    @cnc.event('robot', 'CloseChuck', 'READY')

    @cnc.event('robot', 'MaterialLoad', 'COMPLETE')
    @cnc.event('robot', 'MaterialLoad', 'READY')

    @cnc.statemachine.state.should == :cycle_start
    @cnc.event('cnc', 'Execution', 'READY')
  end
end

RSpec.configure do |c|
  c.include Helpers
end
