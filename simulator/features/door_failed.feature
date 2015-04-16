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

Feature: Machine Tool Door Failed
  The robot has request the machine tool open its door
  but the door fails

  Background: Machine Tool and Robot are operational
    Given Devices are in initial state

  Scenario: Cnc Can not Close Its Door
    Given robot MaterialHandlerInterface MaterialLoad becomes Active
    Then material load state should be processing
    And Chuck is closed

    When robot DoorInterface Close becomes Active
    Then cnc CloseDoor should be Active
    And cnc Door DoorState becomes Unlatched

    When cnc DoorInterface Close becomes Failure
    Then close door state should be fail
    And cnc CloseDoor should be Fail
    And cnc DoorState should be Unlatched
    And cnc should have fault
    And machine state should be fault
    And cnc fault should have code "Cnc::CloseDoor"

  Scenario: Cnc Can not Close Door and then Recovers
    Given robot MaterialHandlerInterface MaterialLoad becomes Active
    Then material load state should be processing

    When robot DoorInterface Close becomes Active
    Then cnc CloseDoor should be Active
    And cnc Door DoorState becomes Unlatched

    When cnc DoorInterface Close becomes Failure
    And close door state should be fail
    And robot DoorInterface Close becomes Fail
    Then cnc CloseDoor should be Not_Ready

    When robot DoorInterface Close becomes Ready
    Then cnc CloseDoor should be Not_Ready
    And machine state should be fault

    When faults are cleared
    And cnc is reset
    Then machine state should be loading
    And cnc CloseDoor should be READY

  Scenario: Simulate door failure
    Given robot MaterialHandlerInterface MaterialLoad becomes Active
    And simulate fail close_door
    Then material load state should be processing

    When robot DoorInterface Close becomes Active
    Then cnc CloseDoor should be Fail
    And cnc fault should have code "Cnc::CloseDoor"

