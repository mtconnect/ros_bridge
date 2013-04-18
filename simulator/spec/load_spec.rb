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
    @load = Cnc::MaterialLoad.new(@cnc)

    @load.fail_timeout = 1.0
    @load.processing_timeout = 1.0
  end

  it "should not be ready" do
    @load.statemachine.state.should == :not_ready
    @material_load.value.should == "NOT_READY"
  end

  it "should transition to ready when ready" do
    @load.statemachine.ready
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
    @load.statemachine.fail
    @load.statemachine.state.should == :fail
    @material_load.value.should == "FAIL"
  end

  it "should return to not_ready when back to not_ready" do
    @load.statemachine.ready
    @load.statemachine.activate
    @load.statemachine.active
    @load.statemachine.fail
    @load.statemachine.not_ready
    @load.statemachine.state.should == :not_ready
    @material_load.value.should == "NOT_READY"
  end

  it "should transition to not_ready after timeout" do
    @load.statemachine.ready
    @load.statemachine.activate
    @load.statemachine.active
    @load.statemachine.fail

    sleep 1.2

    @load.statemachine.state.should == :not_ready
    @material_load.value.should == "NOT_READY"
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
    @load.statemachine.state.should == :not_ready
    @material_load.value.should == "NOT_READY"
  end
end