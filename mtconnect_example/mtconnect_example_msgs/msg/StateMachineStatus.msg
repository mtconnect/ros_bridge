# The StateMachineStatus message contains status information 
# about the current running state machine.

# The header frame ID is not used
Header header

# Current state (enumerated type)
int32 state

# Current state (string)
string state_name
