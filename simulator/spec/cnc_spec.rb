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

require 'spec_helper'
require 'cnc'

describe "Cnc" do
  before(:each) do
    control = double("socket")
    control.stub!(:puts) { }
    @cnc = Cnc::CncContext.new(control)
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
      @cnc.has_material = false
      @cnc.event('cnc', 'Controller', 'ControllerMode', 'AUTOMATIC')
      @cnc.event('robot', 'Device', 'Availability', 'AVAILABLE')
      @cnc.event('robot', 'Controller', 'ControllerMode', 'AUTOMATIC')
      @cnc.event('robot', 'Controller', 'Execution', 'ACTIVE')
      @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
      @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
      @cnc.event('robot', 'DoorInterface', 'Open', 'READY')
      @cnc.event('robot', 'DoorInterface', 'Close', 'READY')
      @cnc.event('robot', 'ChuckInterface', 'Open', 'READY')
      @cnc.event('robot', 'ChuckInterface', 'Close', 'READY')

      @cnc.statemachine.state.should == :loading
    end

    context "when loading material" do
      before(:each) do
        @cnc.event('cnc', 'Controller', 'ControllerMode', 'AUTOMATIC')
        @cnc.event('robot', 'Device', 'Availability', 'AVAILABLE')
        @cnc.event('robot', 'Controller', 'ControllerMode', 'AUTOMATIC')
        @cnc.event('robot', 'Controller', 'Execution', 'ACTIVE')
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
        @cnc.event('robot', 'DoorInterface', 'Open', 'READY')
        @cnc.event('robot', 'DoorInterface', 'Close', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Close', 'READY')
      end

      it "should have the material load active" do
        @cnc.material_load.value.should == 'ACTIVE'
      end

      it "should open the door when open door becomes active" do
        @cnc.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
        sleep 1.5
        @cnc.door_state.value.should == 'CLOSED'
      end

      it "should close the chuck when close chuck becomes active" do
        @cnc.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
        sleep 1.5
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.close_chuck.value.should == 'COMPLETE'
      end

      it "should begin a part after material load is complete" do
        @cnc.has_material.should be_false
        @cnc.statemachine.state.should == :loading

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')

        sleep 1.5

        @cnc.door_state.value.should == 'OPEN'
        @cnc.chuck_state.value.should == 'OPEN'
        @cnc.event('robot', 'DoorInterface', 'Open', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'READY')

        @cnc.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
        @cnc.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')

        sleep 1.5

        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'

        @cnc.event('robot', 'DoorInterface', 'Open', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'READY')

        @cnc.statemachine.state.should == :loading

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

        @cnc.has_material.should be_true
        @cnc.statemachine.state.should == :cycle_start

        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.material_unload.value.should == 'NOT_READY'
        
        sleep 1.2

        @cnc.statemachine.state.should == :unloading

        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.material_unload.value.should == 'ACTIVE'
      end

      it 'should unload when after the machine has cut a part' do
        unloading
        @cnc.statemachine.state.should == :unloading

        @cnc.material_unload.value.should == 'ACTIVE'
        @cnc.material_load.value.should == 'NOT_READY'

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
        @cnc.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')
        sleep 1.2
        @cnc.event('robot', 'DoorInterface', 'Open', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'READY')
        @cnc.door_state.value.should == 'OPEN'
        @cnc.chuck_state.value.should == 'OPEN'

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'COMPLETE')
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')

        @cnc.statemachine.state.should == :loading

        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.material_unload.value.should == 'NOT_READY'
      end

      it "should be not ready when machine goes into manual mode" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'Controller', 'ControllerMode', 'MANUAL')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :not_ready
      end

      it "should be not ready when the execution state becomes ready" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'Controller', 'Execution', 'READY')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :not_ready
      end

      it "should be not ready when a fault occurs" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'Device', 'SYSTEM', 'Fault', '1', 'failure')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :fault
      end

      it "should be ready after a fault clears" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'Device', 'SYSTEM', 'Fault', '1', 'failure')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :fault
        @cnc.event('robot', 'Device', 'SYSTEM', 'Normal')
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.statemachine.state.should == :loading
      end

      it "should be ready after a single fault clears" do
        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.event('robot', 'Device', 'SYSTEM', 'Fault', '1', 'failure')
        @cnc.material_load.value.should == 'NOT_READY'
        @cnc.statemachine.state.should == :fault
        @cnc.event('robot', 'Device', 'SYSTEM', 'Normal', '1', '')

        @cnc.material_load.value.should == 'ACTIVE'
        @cnc.statemachine.state.should == :loading
      end

      it "should fail if the chuck is open when it tries to cycle start" do
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc', 'Rotary', 'ChuckState', 'OPEN')
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'OPEN'
        @cnc.event('robot', 'DoorInterface', 'Close', 'READY')

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :fault
      end

      it "should fail if the door is open when it tries to cycle start" do
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc', 'Rotary', 'ChuckState', 'CLOSED')
        @cnc.door_state.value.should == 'OPEN'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('robot', 'ChuckInterface', 'Close', 'READY')

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

        @cnc.statemachine.state.should == :fault
      end

      it "should fail a material load if the robot fails" do
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
        @cnc.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc', 'Rotary', 'ChuckState', 'CLOSED')
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('robot', 'DoorInterface', 'Close', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Close', 'READY')

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')
        @cnc.statemachine.state.should == :loading
        @cnc.material_load.value.should == 'FAIL'

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
        @cnc.statemachine.state.should == :idle
        @cnc.material_load_interface.state.should == :ready
        @cnc.material_load.value.should == 'READY'
        @cnc.material_unload.value.should == 'READY'

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
        @cnc.statemachine.state.should == :unloading
        @cnc.material_unload.value.should == 'ACTIVE'
      end

      it "should fail and become inactive if a material load fails and then goes to not ready" do
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
        @cnc.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc', 'Rotary', 'ChuckState', 'CLOSED')
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'
        @cnc.event('robot', 'DoorInterface', 'Close', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Close', 'READY')

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')
        @cnc.statemachine.state.should == :loading
        @cnc.material_load_interface.state.should == :fail
        @cnc.material_load.value.should == 'FAIL'

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
        @cnc.statemachine.state.should == :idle

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
        @cnc.statemachine.state.should == :unloading

        @cnc.material_unload.value.should == 'ACTIVE'
        @cnc.system.should be_normal
      end

      it "should fail a material unload if the robot fails and return to active when ready" do
        unloading
        @cnc.statemachine.state.should == :unloading

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'FAIL')
        @cnc.statemachine.state.should == :unloading
        @cnc.material_unload_interface.state.should == :fail
        @cnc.material_unload.value.should == 'FAIL'

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
        @cnc.statemachine.state.should == :unloading
        @cnc.material_unload.value.should == 'ACTIVE'
      end

      it "should fail if the load active does not complete in a certain amount of time" do
        @cnc.load_time_limit 1
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
        sleep 1.2

        @cnc.statemachine.state.should == :loading
        @cnc.material_load_interface.state.should == :fail
        @cnc.material_load.value.should == 'FAIL'
      end

      it "should fail if the unload active does not complete in a certain amount of time" do
        @cnc.unload_time_limit 1

        unloading
        @cnc.statemachine.state.should == :unloading

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
        sleep 1.2

        @cnc.statemachine.state.should == :unloading
        @cnc.material_unload_interface.state.should == :fail
        @cnc.material_unload.value.should == 'FAIL'
      end

      it "should fault if the load fail is not resolved in a certain amount of time" do
        @cnc.load_time_limit 1
        @cnc.load_failed_time_limit 1
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
        sleep 1.2

        @cnc.statemachine.state.should == :loading
        @cnc.material_load.value.should == 'FAIL'

        sleep 1.2
        @cnc.statemachine.state.should == :idle
      end

      it "should not fail a load if unload becomes not ready" do
        @cnc.unload_time_limit 1

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
        @cnc.material_unload.value.should == "NOT_READY"

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'NOT_READY')
        @cnc.statemachine.state.should == :loading
      end

      it "should fail execution if fail next is true" do
        @cnc.fail_next('exec', true)
        @cnc.has_material.should be_false
        @cnc.statemachine.state.should == :loading

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
        @cnc.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')

        sleep 1.2

        @cnc.event('cnc','Rotary', 'ChuckState', 'OPEN')
        @cnc.door_state.value.should == 'OPEN'
        @cnc.chuck_state.value.should == 'OPEN'
        @cnc.event('robot', 'DoorInterface', 'Open', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'READY')

        @cnc.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
        @cnc.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')

        sleep 1.2

        @cnc.event('cnc','Rotary', 'ChuckState', 'CLOSED')
        @cnc.door_state.value.should == 'CLOSED'
        @cnc.chuck_state.value.should == 'CLOSED'

        @cnc.event('robot', 'DoorInterface', 'Open', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'READY')

        @cnc.statemachine.state.should == :loading

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

        @cnc.has_material.should be_true
        @cnc.statemachine.state.should == :fault
      end

      it "should not fail an unload if load becomes not ready" do
        unloading
        @cnc.statemachine.state.should == :unloading

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
        @cnc.statemachine.state.should == :unloading
      end

      it "should be operational when material load is not ready (out of material) but can still unload" do
        unloading
        @cnc.statemachine.state.should == :unloading

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
        @cnc.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')
        sleep 1.2
        @cnc.event('cnc','Rotary', 'ChuckState', 'OPEN')
        @cnc.event('robot', 'DoorInterface', 'Open', 'READY')
        @cnc.event('robot', 'ChuckInterface', 'Open', 'READY')
        @cnc.door_state.value.should == 'OPEN'
        @cnc.chuck_state.value.should == 'OPEN'

        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'COMPLETE')
        @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
        @cnc.statemachine.state.should == :loading
        @cnc.material_load.value.should == 'ACTIVE'
      end


      context "and robot is out of material" do
        it "should make material load not ready when it is active and the robot material load is not ready" do
          @cnc.material_load.value.should == 'ACTIVE'
          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', "NOT_READY")
          @cnc.material_load.value.should == 'READY'
        end

        it "should keep material unload ready when it is ready and the robot material load is not ready" do
          @cnc.material_load.value.should == 'ACTIVE'

          @cnc.door_state.value = "CLOSED"
          @cnc.chuck_state.value = "CLOSED"
          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
          
          sleep 1.2

          @cnc.material_unload.value.should == 'ACTIVE'
          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', "NOT_READY")
          @cnc.material_load.value.should == 'NOT_READY'
          @cnc.material_unload.value.should == 'ACTIVE'
        end

        it "should fail if material load is not ready and current state is material load" do
          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')

          @cnc.has_material.should_not be_true

          @cnc.material_load.value.should == 'FAIL'
          @cnc.material_unload.value.should == 'NOT_READY'

          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
          @cnc.material_unload.value.should == 'NOT_READY'
          @cnc.material_load.value.should == 'ACTIVE'
        end

        it "should go back to loading material if it is not available to the robot was reset or filled" do
          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')

          @cnc.has_material.should_not be_true

          @cnc.material_load.value.should == 'FAIL'
          @cnc.material_unload.value.should == 'NOT_READY'

          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
          @cnc.material_unload.value.should == 'NOT_READY'
          @cnc.material_load.value.should == 'READY'

          @cnc.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
          @cnc.material_unload.value.should == 'NOT_READY'
          @cnc.material_load.value.should == 'ACTIVE'
        end
      end
    end
  end
end