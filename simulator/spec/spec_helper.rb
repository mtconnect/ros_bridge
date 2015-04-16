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
    @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
    @cnc.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
    @cnc.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
    sleep 1.2
    @cnc.door_state.value.should == 'CLOSED'
    @cnc.chuck_state.value.should == 'CLOSED'
    @cnc.event('robot', 'DoorInterface', 'Close', 'READY')
    @cnc.event('robot', 'ChuckInterface', 'Close', 'READY')

    @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
    @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

    sleep 1.2
  end
end

RSpec.configure do |c|
  c.include Helpers
end
