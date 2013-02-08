# The RobotStatus message contains status information that is specified
# by the MTConnect standard for CNC machine tools.

# The header frame ID is not used
Header header

# Robot Status
industrial_msgs/TriState avail

# Robot Mode
industrial_msgs/RobotMode mode

# Robot Execution
industrial_msgs/TriState rexec
