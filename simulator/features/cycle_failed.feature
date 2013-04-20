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

Feature: Machine Tool cycle failure
  The robot loads material into the machine tool and then the cycle
  fails

  Background: Cnc begins a cycle
    Given Devices are in initial state
    And robot MaterialLoad becomes Active
    And Chuck is closed
    And Door is closed
    And robot MaterialLoad becomes Complete
    And robot MaterialLoad becomes Ready
    And cnc MaterialLoad should be Not_Ready
    And machine state should be cycle_start

  Scenario: Cycle starts and then alarms out
    Given machine state should be cycle_start
    And cnc Execution is Active

    When cnc faults MOTION_PROGRAM with "Program not loaded"
    And cnc Execution is Stopped
    Then machine state should be fault

  Scenario: Cycle starts and then alarms out and the alarm clears
    Given machine state should be cycle_start
    And cnc Execution is Active

    When cnc faults MOTION_PROGRAM with "Program not loaded"
    And cnc Execution is Stopped
    Then machine state should be fault

    When cnc clears MOTION_PROGRAM
    When cnc is reset
    Then machine state should be unloading
    And cnc MaterialLoad should be Not_Ready
    And cnc MaterialUnload should be Active

