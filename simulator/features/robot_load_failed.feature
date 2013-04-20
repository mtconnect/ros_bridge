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
Feature: Robot Load Material Fails
  As the robot is loading material it fails

  Background: Machine Tool and Robot are operational
    Given Devices are in initial state

  Scenario: Robot can't load material
    Given cnc MaterialLoad should be Active
    And robot MaterialLoad becomes Active
    And Chuck is closed
    And Door is closed

    When robot MaterialLoad becomes Fail
    Then cnc MaterialLoad should be Fail
    And material load state should be fail

    When robot MaterialLoad becomes Not_Ready
    And robot faults ACTUATOR with "Flame"
    And machine state should be idle

  Scenario: Robot can't load material and recovers
    Given cnc MaterialLoad should be Active
    And robot MaterialLoad becomes Active
    And Chuck is closed
    And Door is closed

    When robot MaterialLoad becomes Fail
    Then cnc MaterialLoad should be Fail
    And material load state should be fail

    When robot MaterialLoad becomes Not_Ready
    And robot faults ACTUATOR with "Flame"
    And machine state should be idle

    When robot clears ACTUATOR
    And robot MaterialLoad becomes Ready
    And machine state should be loading
