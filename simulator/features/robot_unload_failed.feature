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

Feature: Robot Unload Material Fails
  As the robot is unloading material it fails

  Background: Machine Tool and Robot are operational
    Given Devices are in initial state
    And robot MaterialLoad becomes Active
    And Chuck is closed
    And Door is closed
    And robot MaterialLoad becomes Complete
    And robot MaterialLoad becomes Ready
    And cnc MaterialLoad should be Not_Ready
    And machine state should be cycle_start

    And cnc Execution becomes Active
    And cnc Execution becomes Ready
    And cnc MaterialUnload should be Active
    And cnc MaterialLoad should be Not_Ready
    And machine state should be unloading

  Scenario: Robot can't unload material
    Given machine state should be unloading

    When robot MaterialUnload becomes Active
    Then material unload state should be processing

    When robot OpenDoor becomes Active
    Then cnc OpenDoor should be Active
    And cnc DoorState should be Unlatched
    And after 1.2 seconds cnc OpenDoor should be Complete
    And cnc DoorState should be Open

    When robot MaterialUnload becomes Fail
    Then cnc MaterialUnload should be Fail
    And material unload state should be fail

    When robot MaterialUnload becomes Not_Ready
    And robot faults ACTUATOR with "Flame"
    And machine state should be idle

  Scenario: Robot can't unload material and recovers
    Given machine state should be unloading

    When robot MaterialUnload becomes Active
    Then material unload state should be processing

    When robot OpenDoor becomes Active
    Then cnc OpenDoor should be Active
    And cnc DoorState should be Unlatched
    And after 1.2 seconds cnc OpenDoor should be Complete
    And cnc DoorState should be Open

    When robot MaterialUnload becomes Fail
    Then cnc MaterialUnload should be Fail
    And material unload state should be fail

    When robot MaterialUnload becomes Not_Ready
    And robot faults ACTUATOR with "Flame"
    And machine state should be idle

    When robot clears ACTUATOR
    And robot MaterialUnload becomes Ready
    And machine state should be unloading
