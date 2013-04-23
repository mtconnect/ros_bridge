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
  class OpenChuck < Response
    def initialize(cnc, control, rel = nil)
      super(cnc, cnc.adapter, cnc.open_chuck, 'chuck', 'OPEN', 'UNLATCHED', rel)
      @control = control
      create_statemachine
    end

    def execute
      @control.puts "* open" unless @simulate
    end

    def response_state
      @parent.cnc_chuck_state
    end
  end

  class CloseChuck < Response
    def initialize(cnc, control, rel = nil)
      super(cnc, cnc.adapter, cnc.close_chuck, 'chuck', 'CLOSED', 'UNLATCHED', rel)
      @control = control
      create_statemachine
    end

    def execute
      @control.puts "* close" unless @simulate
    end

    def response_state
      @parent.cnc_chuck_state
    end
  end
end