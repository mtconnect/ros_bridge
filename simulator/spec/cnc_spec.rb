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
#    limitations under the License.require "rspec"

$: << File.dirname(__FILE__) + '/..'

require 'cnc'

describe "Cnc" do
  before(:each) do
    @cnc = Cnc::CncContext.new
    @cnc.cycle_time = 1
  end

  context "operating" do
    it "should have an initial state of not ready" do
      @cnc.statemachine.state.should == :not_ready
    end

    it "should not transition before it's ready" do
      @cnc.statemachine.activate
      @cnc.statemachine.state.should == :not_ready
    end

    it 'should become ready when the link is enabled, all interfaces are ready, and the robot and machine tool are in automatic' do
      @cnc.event('Availability', 'AVAILABLE')
      @cnc.event('ControllerMode', 'AUTOMATIC')
      @cnc.event('Execution', 'ACTIVE')
      @cnc.event('MaterialLoad', 'READY')
      @cnc.event('MaterialUnload', 'READY')
      @cnc.event('OpenDoor', 'READY')
      @cnc.event('CloseDoor', 'READY')
      @cnc.event('OpenChuck', 'READY')
      @cnc.event('CloseChuck', 'READY')

      @cnc.statemachine.state.should == :material_load
    end

    context "when loading material" do
      before(:each) do
        @cnc.event('Availability', 'AVAILABLE')
        @cnc.event('ControllerMode', 'AUTOMATIC')
        @cnc.event('Execution', 'ACTIVE')
        @cnc.event('MaterialLoad', 'READY')
        @cnc.event('MaterialUnload', 'READY')
        @cnc.event('OpenDoor', 'READY')
        @cnc.event('CloseDoor', 'READY')
        @cnc.event('OpenChuck', 'READY')
        @cnc.event('CloseChuck', 'READY')
      end

      it "should have the material load active" do
        @cnc.material_load.value.should == 'ACTIVE'
      end

      it "should open the door when open door becomes active" do
        @cnc.event('CloseDoor', 'ACTIVE')
        sleep 1.5
        @cnc.door_state.value.should == 'CLOSED'
      end

      it "should open the chuck when open chuck becomes active" do
        @cnc.event('CloseChuck', 'ACTIVE')
        sleep 1.5
        @cnc.chuck_state.value.should == 'CLOSED'
      end

      it "should be begin a part after material load is complete" do
        @cnc.event('MaterialLoad', 'ACTIVE')
        @cnc.event('OpenDoor', 'ACTIVE')
        @cnc.event('OpenChuck', 'ACTIVE')
        sleep 1.2
        @cnc.door_state.value.should == 'OPEN'
        @cnc.chuck_state.value.should == 'OPEN'
        @cnc.event('OpenDoor', 'READY')
        @cnc.event('OpenChuck', 'READY')

        @cnc.event('CloseDoor', 'ACTIVE')
        @cnc.event('CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('OpenDoor', 'READY')
        @cnc.event('OpenChuck', 'READY')

        @cnc.event('MaterialLoad', 'COMPLETE')
        @cnc.event('MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :cycle_start
        sleep 1.2
        @cnc.statemachine.state.should == :material_unload
        @cnc.material_unload.value.should == 'ACTIVE'
        @cnc.material_load.value.should == 'READY'
      end

      it 'should unload when after the machine has cut a part' do
        @cnc.event('MaterialLoad', 'ACTIVE')
        @cnc.event('CloseDoor', 'ACTIVE')
        @cnc.event('CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('CloseDoor', 'READY')
        @cnc.event('CloseChuck', 'READY')

        @cnc.event('MaterialLoad', 'COMPLETE')
        @cnc.event('MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :cycle_start
        sleep 1.2
        @cnc.statemachine.state.should == :material_unload
        @cnc.material_unload.value.should == 'ACTIVE'
        @cnc.material_load.value.should == 'READY'

        @cnc.event('MaterialUnload', 'ACTIVE')
        @cnc.event('OpenDoor', 'ACTIVE')
        @cnc.event('OpenChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('OpenDoor', 'READY')
        @cnc.event('OpenChuck', 'READY')
        @cnc.door_state.value.should == 'OPEN'
        @cnc.chuck_state.value.should == 'OPEN'

        @cnc.event('MaterialUnload', 'COMPLETE')
        @cnc.event('MaterialUnload', 'READY')
        @cnc.statemachine.state.should == :material_load
        @cnc.material_load.value.should == 'ACTIVE'
      end

      it "should be not ready when machine goes into manual mode" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('ControllerMode', 'MANUAL')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :not_ready
      end

      it "should be not ready when the execution state becomes ready" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('Execution', 'READY')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :not_ready
      end

      it "should be not ready when a fault occurs" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('SYSTEM', 'Fault', '1', 'failure')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :fault
      end

      it "should be ready after a fault clears" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('SYSTEM', 'Fault', '1', 'failure')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :fault
        @cnc.event('SYSTEM', 'Normal')
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.statemachine.state.should == :material_load
      end

      it "should be ready after a single fault clears" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('SYSTEM', 'Fault', '1', 'failure')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :fault
        @cnc.event('SYSTEM', 'Normal', '1', '')
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.statemachine.state.should == :material_load
      end

      it "should fail if the chuck is open when it tries to cycle start" do
        @cnc.event('MaterialLoad', 'ACTIVE')
        @cnc.event('CloseDoor', 'ACTIVE')
        sleep 1.2
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'OPEN'
        @cnc.event('CloseDoor', 'READY')

        @cnc.event('MaterialLoad', 'COMPLETE')
        @cnc.event('MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :fault
      end

      it "should fail if the door is open when it tries to cycle start" do
        @cnc.event('MaterialLoad', 'ACTIVE')
        @cnc.event('CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.door_state.value.should == 'OPEN'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('CloseChuck', 'READY')

        @cnc.event('MaterialLoad', 'COMPLETE')
        @cnc.event('MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :fault
      end

      it "should fail a material load if the robot fails" do
        @cnc.event('MaterialLoad', 'ACTIVE')
        @cnc.event('CloseDoor', 'ACTIVE')
        @cnc.event('CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('CloseDoor', 'READY')
        @cnc.event('CloseChuck', 'READY')

        @cnc.event('MaterialLoad', 'FAIL')
        @cnc.statemachine.state.should == :material_load_failed
        @cnc.material_load.value.should == 'FAIL'

        @cnc.event('MaterialLoad', 'READY')
        @cnc.statemachine.state.should == :material_load
        @cnc.material_load.value.should == 'ACTIVE'
      end

      it "should fail and become inactive if a material load fails and then goes to not ready" do
        @cnc.event('MaterialLoad', 'ACTIVE')
        @cnc.event('CloseDoor', 'ACTIVE')
        @cnc.event('CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('CloseDoor', 'READY')
        @cnc.event('CloseChuck', 'READY')

        @cnc.event('MaterialLoad', 'FAIL')
        @cnc.statemachine.state.should == :material_load_failed
        @cnc.material_load.value.should == 'FAIL'

        @cnc.event('MaterialLoad', 'NOT_READY')
        @cnc.statemachine.state.should == :not_ready
        @cnc.material_load.value.should == 'NOT_READY'
      end

      it "should fail a material unload if the robot fails and return to active when ready" do
        @cnc.event('MaterialLoad', 'ACTIVE')
        @cnc.event('CloseDoor', 'ACTIVE')
        @cnc.event('CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('CloseDoor', 'READY')
        @cnc.event('CloseChuck', 'READY')

        @cnc.event('MaterialLoad', 'COMPLETE')
        @cnc.event('MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :cycle_start
        sleep 1.2
        @cnc.statemachine.state.should == :material_unload

        @cnc.event('MaterialUnload', 'ACTIVE')

        @cnc.event('MaterialUnload', 'FAIL')
        @cnc.statemachine.state.should == :material_unload_failed
        @cnc.material_unload.value.should == 'FAIL'

        @cnc.event('MaterialUnload', 'READY')
        @cnc.statemachine.state.should == :material_unload
        @cnc.material_unload.value.should == 'ACTIVE'
      end

      it "should fail if the load active does not complete in a certain amount of time" do
        @cnc.load_time_limit = 1
        @cnc.event('MaterialLoad', 'ACTIVE')
        sleep 1.2

        @cnc.statemachine.state.should == :material_load_failed
      end

      it "should fail if the unload active does not complete in a certain amount of time" do
        @cnc.unload_time_limit = 1

        @cnc.event('MaterialLoad', 'ACTIVE')
        @cnc.event('CloseDoor', 'ACTIVE')
        @cnc.event('CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('CloseDoor', 'READY')
        @cnc.event('CloseChuck', 'READY')

        @cnc.event('MaterialLoad', 'COMPLETE')
        @cnc.event('MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :cycle_start
        sleep 1.2
        @cnc.statemachine.state.should == :material_unload

        @cnc.event('MaterialUnload', 'ACTIVE')
        sleep 1.2

        @cnc.statemachine.state.should == :material_unload_failed
      end

    end
  end
end