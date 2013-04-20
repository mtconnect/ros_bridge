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

require 'door'
require 'data_item'

# We will only test open door since the code is exactly the same for all responses. close door
# open chuck, and close chuck will be tested if they differ in the future from the generic response.


describe "Response" do
  context "OpenDoor" do
    before(:each) do
      @cnc = double("cnc")
      @door_state = MTConnect::DataItem.new('door_state')
      @door_state.value = 'CLOSED'
      @open_door = MTConnect::DataItem.new('open_door')
      @open_door.value = 'UNAVAILABLE'
      @adapter = double('adapter')
      @adapter.stub(:gather).and_yield
      @cnc.stub(:open_door) { @open_door }
      @cnc.stub(:door_state) { @door_state }
      @cnc.stub(:adapter) { @adapter }
      @cnc.stub!(:failed) { }
      @cnc.stub!(:completed) { }
      @door = Cnc::OpenDoor.new(@cnc)
    end

    context "operating correctly" do
      it "should be NOT_READY when it starts" do
        @door.statemachine.state.should == :not_ready
        @open_door.value.should == 'NOT_READY'
        @door_state.value == 'CLOSED'
      end

      it "should become ready when request is ready" do
        @door.statemachine.state.should == :not_ready
        @door.statemachine.ready
        @door.statemachine.state.should == :ready
        @open_door.value.should == 'READY'
      end

      it 'should transition to active when request becomes active' do
        @door.statemachine.ready
        @door.statemachine.active
        @door_state.value.should == 'UNLATCHED'
        @door.statemachine.state.should == :active
        @open_door.value.should == 'ACTIVE'
      end

      it 'should transition from active to complete after one second' do
        @door.statemachine.ready
        @door_state.value.should == 'CLOSED'
        @door.statemachine.active
        @door.statemachine.state.should == :active
        @open_door.value.should == 'ACTIVE'
        sleep 1.5
        @door.statemachine.state.should == :complete
        @open_door.value.should == 'COMPLETE'
        @door_state.value.should == 'OPEN'
      end

      it 'should transition from complete to ready when request becomes ready' do
        @door.statemachine.ready
        @door_state.value = 'OPEN'
        @door.statemachine.active
        @door.statemachine.state.should == :complete
        @open_door.value.should == 'COMPLETE'
        @door.statemachine.ready
        @door.statemachine.state.should == :ready
        @open_door.value.should == 'READY'
        @door_state.value.should == 'OPEN'
      end

      it 'should transition to NOT_READY when request becomes not ready' do
        @door.statemachine.ready
        @door.statemachine.not_ready
        @open_door.value.should == 'NOT_READY'
      end
    end

    context 'operating incorrectly' do
      it 'should fail if not ready and receives an active from request' do
        @door.fail_reset_delay = 1.0
        @door.statemachine.state.should == :not_ready
        @open_door.value.should == 'NOT_READY'
        @door.statemachine.active
        @door.statemachine.state.should == :fail
        @open_door.value.should == 'FAIL'
        sleep 1.5
        @door.statemachine.state.should == :not_ready
        @open_door.value.should == 'NOT_READY'
      end

      it "should not transition to ready if the cnc is disabled" do
        @door.fail_reset_delay = 1.0
        @door.deactivate
        @door.statemachine.state.should == :not_ready
        @open_door.value.should == 'NOT_READY'
        @door.statemachine.active
        @door.statemachine.state.should == :fail
        @open_door.value.should == 'FAIL'
        sleep 1.5
        @door.statemachine.state.should == :not_ready
        @open_door.value.should == 'NOT_READY'

        @door.statemachine.ready
        @door.statemachine.state.should == :not_ready
        @open_door.value.should == 'NOT_READY'
      end

      context 'when active' do
        before(:each) do
          @door.statemachine.ready
          @door.statemachine.active
          @door.statemachine.state.should == :active
        end

        it 'should fail if active and request fails' do
          @door.statemachine.fail
          @door.statemachine.state.should == :fail
          @open_door.value.should == 'FAIL'
        end

        it 'should fail if active and request becomes unavailable' do
          @door.statemachine.unavailable
          @door.statemachine.state.should == :fail
          @open_door.value.should == 'FAIL'
        end

        it 'should fail if active and request fails' do
          @door.statemachine.not_ready
          @door.statemachine.state.should == :fail
          @open_door.value.should == 'FAIL'
        end
      end
    end

    context 'with a related interface' do
      before(:each) do
        @related = double('CloseDoor')
        @related.stub(:related).and_return(true)
        @interface = double('interface')
        @related.stub(:interface) { @interface }
        @door.related = @related
      end

      it 'should succeed if related is not active' do
        @interface.stub(:value).and_return('READY')
        @door.statemachine.ready
        @door.statemachine.active
        sleep 1.5
        @door.statemachine.state.should == :complete
        @open_door.value.should == 'COMPLETE'
      end

      it 'should fail if related is active' do
        @interface.stub(:value).and_return('ACTIVE')
        @door.statemachine.ready
        @door.statemachine.active
        @door.statemachine.state.should == :fail
        @open_door.value.should == 'FAIL'
      end
    end
  end
end