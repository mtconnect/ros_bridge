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
Feature: Robot Disconnects
  The communications between the Robot and the Cnc fails. If we disconnect in the
  middle of performing an action, then we fault since we are left in a precarious
  state (is this true or can we complete the action?).

  Background: Machine Tool and Robot are operational
    Given Devices are in initial state

  Scenario: Loading Material and Robot Disconnects while Closing Chuck
    Given cnc MaterialLoad should be Active
    When robot MaterialHandlerInterface MaterialLoad becomes Active
    Then material load state should be processing

    And cnc DoorState should be Open
    And cnc Rotary ChuckState becomes Open
    And cnc ChuckState should be Open
    Then machine state should be loading

    When robot ChuckInterface Close becomes Active
    And robot Device Availability becomes Unavailable
    And robot MaterialHandlerInterface MaterialLoad becomes Unavailable
    And robot MaterialHandlerInterface MaterialUnload becomes Unavailable
    And robot ChuckInterface Close becomes Unavailable
    And robot ChuckInterface Open becomes Unavailable
    And robot DoorInterface Close becomes Unavailable
    And robot DoorInterface Open becomes Unavailable

    Then machine state should be fault
    And cnc CloseChuck should be Fail
    And after 5 seconds cnc CloseChuck should be not_ready

    Given machine state should be fault
    And cnc is reset
    Then machine state should be not_ready

  Scenario: Loading Material and Robot Disconnects before loading
    Given cnc MaterialLoad should be Active
    When robot MaterialHandlerInterface MaterialLoad becomes Active
    Then material load state should be processing

    And cnc DoorState should be Open
    And cnc Rotary ChuckState becomes Open
    And cnc ChuckState should be Open
    Then machine state should be loading

    When robot Device Availability becomes Unavailable
    And robot MaterialHandlerInterface MaterialLoad becomes Unavailable
    And robot MaterialHandlerInterface MaterialUnload becomes Unavailable
    And robot ChuckInterface Close becomes Unavailable
    And robot ChuckInterface Open becomes Unavailable
    And robot DoorInterface Close becomes Unavailable
    And robot DoorInterface Open becomes Unavailable

    Then machine state should be not_ready
    And cnc MaterialLoad should be not_ready
