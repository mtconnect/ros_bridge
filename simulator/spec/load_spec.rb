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

$: << File.dirname(__FILE__) + '/..'

require 'data_item'
require 'material'

describe "MaterialLoad" do
  before(:each) do
    @cnc = double("cnc")
    @material_load = MTConnect::DataItem.new('material_load')
    @material_load.value = 'UNAVAILABLE'
    @adapter = double('adapter')
    @adapter.stub(:gather).and_yield
    @cnc.stub(:material_load) { @material_load }
    @cnc.stub(:adapter) { @adapter }
    @cnc.stub(:completed) { }
    @cnc.stub(:failed) { }
    @load = Cnc::MaterialLoad.new(@cnc)

    @load.fail_time_limit = 1.0
    @load.processing_time_limit = 1.0
  end

  it "should not be ready" do
    @load.statemachine.state.should == :not_ready
    @material_load.value.should == "NOT_READY"
  end

  it "should transition to ready when idle" do
    @load.statemachine.idle
    @load.statemachine.state.should == :ready
    @material_load.value.should == "READY"
  end

  it "should transition to active when activated" do
    @load.statemachine.ready
    @load.statemachine.activate
    @load.statemachine.state.should == :active
    @material_load.value.should == "ACTIVE"
  end

  it "should start processing when response becomes active" do
    @load.statemachine.ready
    @load.statemachine.activate
    @load.statemachine.active
    @load.statemachine.state.should == :processing
    @material_load.value.should == "ACTIVE"
  end

  it "should become not_ready processing when complete" do
    @load.statemachine.ready
    @load.statemachine.activate
    @load.statemachine.active
    @load.statemachine.complete
    @load.statemachine.state.should == :not_ready
    @material_load.value.should == "NOT_READY"
  end

  it "should fail when processing fails" do
    @load.statemachine.ready
    @load.statemachine.activate
    @load.statemachine.active
    @load.statemachine.failure
    @load.statemachine.state.should == :fail
    @material_load.value.should == "FAIL"
  end

  it "should return to not_ready when back to not_ready" do
    @load.statemachine.ready
    @load.statemachine.activate
    @load.statemachine.active
    @load.statemachine.failure
    @load.statemachine.not_ready
    @load.statemachine.state.should == :ready
    @material_load.value.should == "READY"
  end

  it "should not be active if the response is not ready" do
    @load.statemachine.ready
    @load.statemachine.activate
    @material_load.value.should == "ACTIVE"

    @load.statemachine.not_ready
    @material_load.value.should == "READY"
  end


  it "should not return to active once the response is ready again" do
    @load.statemachine.ready
    @load.statemachine.activate
    @material_load.value.should == "ACTIVE"

    @load.statemachine.not_ready
    @material_load.value.should == "READY"

    @load.statemachine.ready
    @material_load.value.should == "ACTIVE"
  end


  it "should transition to not_ready after timeout" do
    @load.statemachine.ready
    @load.statemachine.activate
    @load.statemachine.active
    @load.statemachine.failure

    sleep 1.2

    @load.statemachine.state.should == :ready
    @material_load.value.should == "READY"
  end

  it "should fail processing after processing timeout" do
    @load.statemachine.ready
    @load.statemachine.activate
    @load.statemachine.active
    @load.statemachine.state.should == :processing

    sleep 1.2

    @load.statemachine.state.should == :fail
    @material_load.value.should == "FAIL"

    @load.statemachine.not_ready
    @load.statemachine.state.should == :ready
    @material_load.value.should == "READY"
  end
end