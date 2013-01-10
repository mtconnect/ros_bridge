# The CncStatus message contains status information that is specified
# by the MTConnect standard for CNC machine tools.

# The header frame ID is not used
Header header


# Door State: Opened/Closed
industrial_msgs/TriState door_state

# Chuck State: Opened/Closed
industrial_msgs/TriState chuck_state

# Responses to Robot Requests for Door and Chuck
# Open Door: Active/Complete/Ready
industrial_msgs/TriState open_door

# Close Door: Active/Complete/Ready
industrial_msgs/TriState close_door

# Open Chuck: Active/Complete/Ready
industrial_msgs/TriState open_chuck

# Close Chuck: Active/Complete/Ready
industrial_msgs/TriState close_chuck

