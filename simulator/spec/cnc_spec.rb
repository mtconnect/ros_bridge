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
      @cnc.event('cnc', 'ControllerMode', 'AUTOMATIC')
      @cnc.event('robot', 'Availability', 'AVAILABLE')
      @cnc.event('robot', 'ControllerMode', 'AUTOMATIC')
      @cnc.event('robot', 'Execution', 'ACTIVE')
      @cnc.event('robot', 'MaterialLoad', 'READY')
      @cnc.event('robot', 'MaterialUnload', 'READY')
      @cnc.event('robot', 'OpenDoor', 'READY')
      @cnc.event('robot', 'CloseDoor', 'READY')
      @cnc.event('robot', 'OpenChuck', 'READY')
      @cnc.event('robot', 'CloseChuck', 'READY')

      @cnc.statemachine.state.should == :material_load
    end

    context "when loading material" do
      before(:each) do
        @cnc.event('cnc', 'ControllerMode', 'AUTOMATIC')
        @cnc.event('robot', 'Availability', 'AVAILABLE')
        @cnc.event('robot', 'ControllerMode', 'AUTOMATIC')
        @cnc.event('robot', 'Execution', 'ACTIVE')
        @cnc.event('robot', 'MaterialLoad', 'READY')
        @cnc.event('robot', 'MaterialUnload', 'READY')
        @cnc.event('robot', 'OpenDoor', 'READY')
        @cnc.event('robot', 'CloseDoor', 'READY')
        @cnc.event('robot', 'OpenChuck', 'READY')
        @cnc.event('robot', 'CloseChuck', 'READY')
      end

      it "should have the material load active" do
        @cnc.material_load.value.should == 'ACTIVE'
      end

      it "should open the door when open door becomes active" do
        @cnc.event('robot', 'CloseDoor', 'ACTIVE')
        sleep 1.5
        @cnc.door_state.value.should == 'CLOSED'
      end

      it "should open the chuck when open chuck becomes active" do
        @cnc.event('robot', 'CloseChuck', 'ACTIVE')
        @cnc.event('cnc', 'ChuckState', 'CLOSED')
        @cnc.cnc_chuck_state.should == 'CLOSED'
        @cnc.close_chuck.value == 'COMPLETE'
      end

      it "should begin a part after material load is complete" do
        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'OpenDoor', 'ACTIVE')
        @cnc.event('robot', 'OpenChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc','ChuckState', 'OPEN')
        @cnc.door_state.value.should == 'OPEN'
        @cnc.cnc_chuck_state.should == 'OPEN'
        @cnc.event('robot', 'OpenDoor', 'READY')
        @cnc.event('robot', 'OpenChuck', 'READY')

        @cnc.event('robot', 'CloseDoor', 'ACTIVE')
        @cnc.event('robot', 'CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc','ChuckState', 'CLOSED')
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.cnc_chuck_state.should == 'CLOSED'
        @cnc.event('robot', 'OpenDoor', 'READY')
        @cnc.event('robot', 'OpenChuck', 'READY')

        @cnc.event('robot', 'MaterialLoad', 'COMPLETE')
        @cnc.event('robot', 'MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :cycle_start
        @cnc.event('cnc', 'Execution', 'READY')
        @cnc.statemachine.state.should == :material_unload
        @cnc.material_unload.value.should == 'ACTIVE'
        @cnc.material_load.value.should == 'READY'
      end

      it 'should unload when after the machine has cut a part' do
        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'CloseDoor', 'ACTIVE')
        @cnc.event('robot', 'CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc','ChuckState', 'CLOSED')
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.cnc_chuck_state.should == 'CLOSED'
        @cnc.event('robot', 'CloseDoor', 'READY')
        @cnc.event('robot', 'CloseChuck', 'READY')

        @cnc.event('robot', 'MaterialLoad', 'COMPLETE')
        @cnc.event('robot', 'MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :cycle_start
        @cnc.event('cnc', 'Execution', 'READY')
        @cnc.statemachine.state.should == :material_unload
        @cnc.material_unload.value.should == 'ACTIVE'
        @cnc.material_load.value.should == 'READY'

        @cnc.event('robot', 'MaterialUnload', 'ACTIVE')
        @cnc.event('robot', 'OpenDoor', 'ACTIVE')
        @cnc.event('robot', 'OpenChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc','ChuckState', 'OPEN')
        @cnc.event('robot', 'OpenDoor', 'READY')
        @cnc.event('robot', 'OpenChuck', 'READY')
        @cnc.door_state.value.should == 'OPEN'
        @cnc.cnc_chuck_state.should == 'OPEN'

        @cnc.event('robot', 'MaterialUnload', 'COMPLETE')
        @cnc.event('robot', 'MaterialUnload', 'READY')
        @cnc.statemachine.state.should == :material_load
        @cnc.material_load.value.should == 'ACTIVE'
      end

      it "should be not ready when machine goes into manual mode" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'ControllerMode', 'MANUAL')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :not_ready
      end

      it "should be not ready when the execution state becomes ready" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'Execution', 'READY')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :not_ready
      end

      it "should be not ready when a fault occurs" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'SYSTEM', 'Fault', '1', 'failure')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :fault
      end

      it "should be ready after a fault clears" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'SYSTEM', 'Fault', '1', 'failure')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :fault
        @cnc.event('robot', 'SYSTEM', 'Normal')
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.statemachine.state.should == :material_load
      end

      it "should be ready after a single fault clears" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'SYSTEM', 'Fault', '1', 'failure')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :fault
        @cnc.event('robot', 'SYSTEM', 'Normal', '1', '')
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.statemachine.state.should == :material_load
      end

      it "should fail if the chuck is open when it tries to cycle start" do
        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'CloseDoor', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc', 'ChuckState', 'OPEN')
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.cnc_chuck_state.should == 'OPEN'
        @cnc.event('robot', 'CloseDoor', 'READY')

        @cnc.event('robot', 'MaterialLoad', 'COMPLETE')
        @cnc.event('robot', 'MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :fault
      end

      it "should fail if the door is open when it tries to cycle start" do
        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc', 'ChuckState', 'CLOSED')
        @cnc.door_state.value.should == 'OPEN'
        @cnc.cnc_chuck_state.should == 'CLOSED'
        @cnc.event('robot', 'CloseChuck', 'READY')

        @cnc.event('robot', 'MaterialLoad', 'COMPLETE')
        @cnc.event('robot', 'MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :fault
      end

      it "should fail a material load if the robot fails" do
        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'CloseDoor', 'ACTIVE')
        @cnc.event('robot', 'CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc', 'ChuckState', 'CLOSED')
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.cnc_chuck_state.should == 'CLOSED'
        @cnc.event('robot', 'CloseDoor', 'READY')
        @cnc.event('robot', 'CloseChuck', 'READY')

        @cnc.event('robot', 'MaterialLoad', 'FAIL')
        @cnc.statemachine.state.should == :material_load_failed
        @cnc.material_load.value.should == 'FAIL'

        @cnc.event('robot', 'MaterialLoad', 'READY')
        @cnc.statemachine.state.should == :material_load
        @cnc.material_load.value.should == 'ACTIVE'
      end

      it "should fail and become inactive if a material load fails and then goes to not ready" do
        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'CloseDoor', 'ACTIVE')
        @cnc.event('robot', 'CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc', 'ChuckState', 'CLOSED')
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.cnc_chuck_state.should == 'CLOSED'
        @cnc.event('robot', 'CloseDoor', 'READY')
        @cnc.event('robot', 'CloseChuck', 'READY')

        @cnc.event('robot', 'MaterialLoad', 'FAIL')
        @cnc.statemachine.state.should == :material_load_failed
        @cnc.material_load.value.should == 'FAIL'

        @cnc.event('robot', 'MaterialLoad', 'NOT_READY')
        @cnc.statemachine.state.should == :material_load
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.system.should be_normal
      end

      it "should fail a material unload if the robot fails and return to active when ready" do
        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'CloseDoor', 'ACTIVE')
        @cnc.event('robot', 'CloseChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc', 'ChuckState', 'CLOSED' )
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.cnc_chuck_state.should == 'CLOSED'
        @cnc.event('robot', 'CloseDoor', 'READY')
        @cnc.event('robot', 'CloseChuck', 'READY')

        @cnc.event('robot', 'MaterialLoad', 'COMPLETE')
        @cnc.event('robot', 'MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :cycle_start
        @cnc.event('cnc', 'Execution', 'READY')
        @cnc.statemachine.state.should == :material_unload

        @cnc.event('robot', 'MaterialUnload', 'ACTIVE')

        @cnc.event('robot', 'MaterialUnload', 'FAIL')
        @cnc.statemachine.state.should == :material_unload_failed
        @cnc.material_unload.value.should == 'FAIL'

        @cnc.event('robot', 'MaterialUnload', 'READY')
        @cnc.statemachine.state.should == :material_unload
        @cnc.material_unload.value.should == 'ACTIVE'
      end

      it "should fail if the load active does not complete in a certain amount of time" do
        @cnc.load_time_limit = 1
        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        sleep 1.2

        @cnc.statemachine.state.should == :material_load_failed
        @cnc.material_load.value.should == 'FAIL'
      end

      it "should fail if the unload active does not complete in a certain amount of time" do
        @cnc.unload_time_limit = 1

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
        @cnc.statemachine.state.should == :material_unload

        @cnc.event('robot', 'MaterialUnload', 'ACTIVE')
        sleep 1.2

        @cnc.statemachine.state.should == :material_unload_failed
        @cnc.material_unload.value.should == 'FAIL'
      end

      it "should fault if the load fail is not resolved in a certain amount of time" do
        @cnc.load_time_limit = 1
        @cnc.load_failed_time_limit = 1
        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        sleep 1.2

        @cnc.statemachine.state.should == :material_load_failed
        @cnc.material_load.value.should == 'FAIL'

        sleep 1.2
        @cnc.statemachine.state.should == :not_ready
      end

      it "should not fail a load if unload becomes not ready" do
        @cnc.unload_time_limit = 1

        @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
        @cnc.material_unload.value.should == "READY"

        @cnc.event('robot', 'MaterialUnload', 'NOT_READY')
        @cnc.statemachine.state.should == :material_load
      end

      it "should not fail an unload if load becomes not ready" do
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
        @cnc.statemachine.state.should == :material_unload

        @cnc.event('robot', 'MaterialUnload', 'ACTIVE')

        @cnc.event('robot', 'MaterialLoad', 'NOT_READY')
        @cnc.statemachine.state.should == :material_unload
      end

      it "should be operational when material load is not ready (out of material) but can still unload" do
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

        # Out of material...
        @cnc.event('robot', 'MaterialLoad', 'NOT_READY')

        @cnc.statemachine.state.should == :cycle_start
        @cnc.event('cnc', 'Execution', 'READY')
        @cnc.statemachine.state.should == :material_unload

        @cnc.event('robot', 'MaterialUnload', 'ACTIVE')
        @cnc.event('robot', 'OpenDoor', 'ACTIVE')
        @cnc.event('robot', 'OpenChuck', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc','ChuckState', 'OPEN')
        @cnc.event('robot', 'OpenDoor', 'READY')
        @cnc.event('robot', 'OpenChuck', 'READY')
        @cnc.door_state.value.should == 'OPEN'
        @cnc.cnc_chuck_state.should == 'OPEN'

        @cnc.event('robot', 'MaterialUnload', 'COMPLETE')
        @cnc.event('robot', 'MaterialUnload', 'READY')
        @cnc.statemachine.state.should == :material_load
        @cnc.material_load.value.should == 'ACTIVE'
      end


      context "and robot is out of material" do
        it "should make material load not ready when it is active and the robot material load is not ready" do
          @cnc.material_load.value.should == 'ACTIVE'
          @cnc.event('robot', 'MaterialLoad', "NOT_READY")
          @cnc.material_load.value.should == 'ACTIVE'
        end

        it "should keep material unload ready when it is ready and the robot material load is not ready" do
          @cnc.material_unload.value.should == 'READY'
          @cnc.event('robot', 'MaterialLoad', "NOT_READY")
          @cnc.material_unload.value.should == 'READY'
        end

        it "should fail if material load is not ready and current state is material load" do
          @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
          @cnc.event('robot', 'MaterialLoad', 'FAIL')
          @cnc.material_load.value.should == 'FAIL'
          @cnc.material_unload.value.should == 'READY'

          @cnc.event('robot', 'MaterialLoad', 'NOT_READY')
          @cnc.material_unload.value.should == 'READY'
          @cnc.material_load.value.should == 'ACTIVE'
        end

        it "should go back to loading material if it is not available to the robot was reset or filled" do
          @cnc.event('robot', 'MaterialLoad', 'ACTIVE')
          @cnc.event('robot', 'MaterialLoad', 'FAIL')
          @cnc.material_load.value.should == 'FAIL'
          @cnc.material_unload.value.should == 'READY'

          @cnc.event('robot', 'MaterialLoad', 'NOT_READY')
          @cnc.material_unload.value.should == 'READY'
          @cnc.material_load.value.should == 'ACTIVE'

          @cnc.event('robot', 'MaterialLoad', 'READY')
          @cnc.material_unload.value.should == 'READY'
          @cnc.material_load.value.should == 'ACTIVE'
        end
      end
    end
  end
end