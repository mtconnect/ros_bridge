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

require 'chuck'
require 'data_item'

# We will only test open chuck since the code is exactly the same for all responses. close chuck
# open chuck, and close chuck will be tested if they differ in the future from the generic response.

describe "Interface" do
  context "OpenChuck" do
    before(:each) do
      @cnc = double("cnc")
      @cnc_chuck_state = 'CLOSED'
      @open_chuck = MTConnect::DataItem.new('open_chuck')
      @open_chuck.value = 'UNAVAILABLE'
      @adapter = double('adapter')
      @adapter.stub(:gather).and_yield
      @cnc.stub(:open_chuck) { @open_chuck }
      @cnc.stub(:cnc_chuck_state) { @cnc_chuck_state }
      @cnc.stub(:adapter) { @adapter }
      @cnc.stub!(:failed) { }
      @cnc.stub!(:completed) { }
      @chuck = Cnc::OpenChuck.new(@cnc)
    end

    context "operating correctly" do
      it "should be NOT_READY when it starts" do
        @chuck.statemachine.state.should == :not_ready
        @open_chuck.value.should == 'NOT_READY'
        @cnc_chuck_state == 'CLOSED'
      end

      it "should become ready when request is ready" do
        @chuck.statemachine.state.should == :not_ready
        @chuck.statemachine.ready
        @chuck.statemachine.state.should == :ready
        @open_chuck.value.should == 'READY'
      end

      it 'should transition to active when request becomes active' do
        @chuck.statemachine.ready
        @chuck.statemachine.active
        @cnc_chuck_state = 'UNLATCHED'
        @chuck.statemachine.chuck_unlatched
        @chuck.statemachine.state.should == :active
        @open_chuck.value.should == 'ACTIVE'
      end

      it 'should transition from active to complete after one second' do
        @chuck.statemachine.ready
        @cnc_chuck_state == 'CLOSED'
        @chuck.statemachine.active
        @chuck.statemachine.state.should == :active
        @open_chuck.value.should == 'ACTIVE'
        @cnc_chuck_state = 'UNLATCHED'
        @chuck.statemachine.chuck_unlatched
        @cnc_chuck_state = 'OPEN'
        @chuck.statemachine.chuck_open
        @chuck.statemachine.state.should == :complete
        @open_chuck.value.should == 'COMPLETE'
        @cnc_chuck_state.should == 'OPEN'
      end

      it 'should transition from complete to ready when request becomes ready' do
        @chuck.statemachine.ready
        @chuck.statemachine.active
        @cnc_chuck_state = 'UNLATCHED'
        @chuck.statemachine.chuck_unlatched
        @cnc_chuck_state = 'OPEN'
        @chuck.statemachine.chuck_open
        @chuck.statemachine.state.should == :complete
        @open_chuck.value.should == 'COMPLETE'
        @chuck.statemachine.ready
        @chuck.statemachine.state.should == :ready
        @open_chuck.value.should == 'READY'
        @cnc_chuck_state.should == 'OPEN'
      end

      it 'should transition to NOT_READY when request becomes not ready' do
        @chuck.statemachine.ready
        @chuck.statemachine.not_ready
        @open_chuck.value.should == 'NOT_READY'
      end
    end

    context 'operating incorrectly' do
      it 'should fail if not ready and receives an active from request' do
        @chuck.fail_reset_delay = 1.0
        @chuck.statemachine.state.should == :not_ready
        @open_chuck.value.should == 'NOT_READY'
        @chuck.statemachine.active
        @chuck.statemachine.state.should == :fail
        @open_chuck.value.should == 'FAIL'
        sleep 1.5
        @chuck.statemachine.state.should == :not_ready
        @open_chuck.value.should == 'NOT_READY'
      end

      it "should not transition to ready if the cnc is disabled" do
        @chuck.fail_reset_delay = 1.0
        @chuck.deactivate
        @chuck.statemachine.state.should == :not_ready
        @open_chuck.value.should == 'NOT_READY'
        @chuck.statemachine.active
        @chuck.statemachine.state.should == :fail
        @open_chuck.value.should == 'FAIL'
        sleep 1.5
        @chuck.statemachine.state.should == :not_ready
        @open_chuck.value.should == 'NOT_READY'

        @chuck.statemachine.ready
        @chuck.statemachine.state.should == :not_ready
        @open_chuck.value.should == 'NOT_READY'
      end

      context 'when active' do
        before(:each) do
          @chuck.statemachine.ready
          @chuck.statemachine.active
          @chuck.statemachine.state.should == :active
        end

        it 'should fail if active and request fails' do
          @chuck.statemachine.fail
          @chuck.statemachine.state.should == :fail
          @open_chuck.value.should == 'FAIL'
        end

        it 'should fail if active and request becomes unavailable' do
          @chuck.statemachine.unavailable
          @chuck.statemachine.state.should == :fail
          @open_chuck.value.should == 'FAIL'
        end

        it 'should fail if active and request fails' do
          @chuck.statemachine.not_ready
          @chuck.statemachine.state.should == :fail
          @open_chuck.value.should == 'FAIL'
        end
      end
    end

    context 'with a related interface' do
      before(:each) do
        @related = double('CloseDoor')
        @related.stub(:related).and_return(true)
        @interface = double('interface')
        @related.stub(:interface) { @interface }
        @chuck.related = @related
      end

      it 'should succeed if related is not active' do
        @interface.stub(:value).and_return('READY')
        @chuck.statemachine.ready
        @chuck.statemachine.active
        @cnc_chuck_state = 'UNLATCHED'
        @chuck.statemachine.chuck_unlatched
        @cnc_chuck_state = 'OPEN'
        @chuck.statemachine.chuck_open
        @chuck.statemachine.state.should == :complete
        @open_chuck.value.should == 'COMPLETE'
      end

      it 'should fail if related is active' do
        @interface.stub(:value).and_return('ACTIVE')
        @chuck.statemachine.ready
        @chuck.statemachine.active
        @chuck.statemachine.state.should == :fail
        @open_chuck.value.should == 'FAIL'
      end
    end
  end
end