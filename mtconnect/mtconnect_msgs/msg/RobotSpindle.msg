# The RobotStatus message contains status information that is specified
# by the MTConnect standard for CNC machine tools.

# The header frame ID is not used
Header header

# Robot System Status
industrial_msgs/TriState c_unclamp

# Robot Spindle Interlock
industrial_msgs/TriState s_inter

