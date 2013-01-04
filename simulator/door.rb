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

require 'rubygems'
require 'statemachine'
require 'mtc_context'
require 'response'

module Cnc
  class OpenDoor < Response
    def initialize(cnc, rel = nil)
      super(cnc.adapter, cnc.open_door, cnc.door_state, 'OPEN', rel)
      create_statemachine
    end


  end

  class CloseDoor < Response
    def initialize(cnc, rel = nil)
      super(cnc.adapter, cnc.close_door, cnc.door_state, 'CLOSED', rel)
      create_statemachine
    end
  end
end