# The CncStatus message contains status information that is specified
# by the MTConnect standard for CNC machine tools.

# The header frame ID is not used
Header header


# Door State: Opened/Closed
industrial_msgs/TriState door_state

# Chuck State: Opened/Closed
industrial_msgs/TriState chuck_state
