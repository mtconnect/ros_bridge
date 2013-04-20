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
Feature: Unload Material
  As a Machine Tool I would like to unload material

  Background: Devices are have completed loading
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

  Scenario: Cnc asks Robot to Unload Material
    Given machine state should be unloading

    When robot MaterialUnload becomes Active
    Then material unload state should be processing

    When robot OpenDoor becomes Active
    Then cnc OpenDoor should be Active
    And cnc DoorState should be Unlatched
    And after 1.2 seconds cnc OpenDoor should be Complete
    And cnc DoorState should be Open

    When robot OpenChuck becomes Active
    And cnc ChuckState becomes Unlatched
    And cnc ChuckState becomes Open
    And cnc OpenChuck becomes Complete
    And robot OpenChuck becomes Ready
    And cnc OpenChuck becomes Ready

    When robot MaterialUnload becomes Complete
    Then cnc MaterialUnload should be Not_Ready
    And cnc MaterialLoad should be Active
    And machine state should be loading
