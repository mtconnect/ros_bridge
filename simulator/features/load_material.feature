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
  The Machine Tool successfully load material

  Background: Machine Tool and Robot are operational
    Given Devices are in initial state

  Scenario: Robot asks Cnc to Open Door
    Given cnc MaterialLoad should be Active
    When robot DoorInterface Open becomes Active
    Then cnc DoorState should be Open

  Scenario: Robot asks Cnc to Open Chuck
    Given cnc MaterialLoad should be Active
    When robot ChuckInterface Open becomes Active
    Then cnc ChuckState should be Open

  Scenario: Cnc asks Robot to Load Material
    Given cnc MaterialLoad should be Active
    When robot MaterialInterface MaterialLoad becomes Active
    Then material load state should be processing

    And cnc DoorState should be Open
    And cnc Rotary ChuckState becomes Open
    And cnc ChuckState should be Open
    Then machine state should be loading

    When robot ChuckInterface Close becomes Active
    Then cnc CloseChuck should be Active
    Then after 1.2 seconds cnc CloseChuck should be Complete
    And cnc ChuckState should be Closed    

    When robot DoorInterface Close becomes Ready
    And robot DoorInterface Close becomes Active
    Then cnc CloseDoor should be Active
    Then after 1.2 seconds cnc CloseDoor should be Complete
    And cnc DoorState should be Closed

    When robot MaterialInterface MaterialLoad becomes Complete
    And robot MaterialInterface MaterialLoad becomes Ready
    Then machine state should be cycle_start
    And cnc MaterialLoad should be Not_Ready
    And robot MaterialInterface MaterialLoad becomes Ready

  Scenario: Cnc fails to start cycle
    Given robot MaterialInterface MaterialLoad becomes Active
    And simulate fail exec

    And Chuck is closed
    And Door is closed
    And robot MaterialInterface MaterialLoad becomes Complete
    And robot MaterialInterface MaterialLoad becomes Ready
    And cnc MaterialLoad should be Not_Ready
    And machine state should be fault
