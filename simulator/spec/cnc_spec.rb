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
  end

  it "should have an initial state of not ready" do
    @cnc.statemachine.state.should == :not_ready
  end

  it "should not transition before it's ready" do
    @cnc.statemachine.activate
    @cnc.statemachine.state.should == :not_ready
  end

  it 'should become ready when the link is enabled, all interfaces are ready, and the robot and machine tool are in automatic' do

  end

  context "when ready to receive parts" do

  end


end