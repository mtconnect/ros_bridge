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

Feature: Robot Out of Material
  The robot begins to load material and signals an alarms
  after it begins and discovers there is not material

  Background: Machine Tool and Robot are operational
    Given Devices are in initial state

  Scenario: Cnc asks Robot to Load Material
    Given robot MaterialInterface MaterialLoad becomes Active
    Then material load state should be processing

    When robot faults Device FILL_LEVEL with "No Material"
    And robot MaterialInterface MaterialLoad becomes Fail
    Then cnc MaterialLoad should be Fail
    And machine state should be loading
    And material load state should be fail

    When robot MaterialInterface MaterialLoad becomes Not_Ready
    Then machine state should be idle
    And cnc MaterialLoad should be Ready

    When robot clears Device FILL_LEVEL
    And robot MaterialInterface MaterialLoad becomes Ready
    Then machine state should be loading
    And cnc MaterialLoad should be Active

