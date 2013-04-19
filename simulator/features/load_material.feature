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

Feature: Load Material
  As a Machine Tool I can ask the robot to load material

  Background: Machine Tool and Robot are operational
    Given Devices are in initial state

  Scenario: Robot asks Cnc to Open Door
    Given cnc MaterialLoad should be Active
    When robot OpenDoor becomes Active
    Then cnc DoorState should be Open

  Scenario: Robot asks Cnc to Open Chuck
    Given cnc MaterialLoad should be Active
    When robot OpenChuck becomes Active
    Then cnc OpenChuck should be Active
    When cnc ChuckState becomes Unlatched
    When cnc ChuckState becomes Open
    Then cnc ChuckState should be Open

  Scenario: Cnc asks Robot to Load Material
    Given cnc MaterialLoad should be Active
    When robot MaterialLoad becomes Active
    Then material load state should be processing

    And cnc DoorState should be Open
    And cnc ChuckState becomes Open
    And cnc ChuckState should be Open
    Then machine state should be loading

    When robot CloseChuck becomes Active
    Then cnc CloseChuck should be Active
    And cnc ChuckState becomes Closed
    And cnc CloseChuck should be Complete
    And robot CloseChuck becomes Ready
    And cnc CloseChuck should be Ready

    When robot CloseDoor becomes Active
    Then cnc CloseDoor should be Active
    Then after 1.2 seconds cnc CloseDoor should be Complete
    And cnc DoorState should be Closed

    When robot MaterialLoad becomes Complete
    And robot MaterialLoad becomes Ready
    Then machine state should be cycle_start
    And cnc MaterialLoad should be Not_Ready
    And robot MaterialLoad becomes Ready

