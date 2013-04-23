#!/bin/sh


# Testing environment variables
echo ${MTCONNECT_AGENT_DIR?Error \$MTCONNECT_AGENT_DIR is not defined, did you export it from your .bashrc file.}

############################### Running agents ######################################
# Setting up agent 1 terminal arguments
TERMINAL1_NAME="Agent_CNC"
WORKING_DIR=$(rosstack find mtconnect)/../simulator/nist
FILE1_PATH=$(rosstack find mtconnect)/../simulator/nist/agent.cfg
COMMAND1="$MTCONNECT_AGENT_DIR/agent debug $FILE1_PATH"
TERMINAL1_CMD="bash -c \" cd $WORKING_DIR; echo $TERMINAL1_NAME; $COMMAND1; exec bash\""

# Setting up agent 2 terminal arguments
TERMINAL2_NAME="Agent_ROBOT"
WORKING_DIR=$(rospack find mtconnect_ros_bridge)/scripts
FILE1_PATH=$WORKING_DIR/agent_robot.cfg
COMMAND1="$MTCONNECT_AGENT_DIR/agent debug $FILE1_PATH"
TERMINAL2_CMD="bash -c \" cd $WORKING_DIR; echo $TERMINAL2_NAME; $COMMAND1; exec bash\""

# Setting up cnc simulator terminal arguments
TERMINAL3_NAME="Cnc Simulator"
WORKING_DIR=$(rosstack find mtconnect)/../simulator
COMMAND1="ruby cnc_simulator.rb http://localhost:5001/Robot"
TERMINAL3_CMD="bash -c \" cd $WORKING_DIR; echo $TERMINAL3_NAME; $COMMAND1; exec bash\""

# Setting up robot simulator terminal arguments
#TERMINAL4_NAME="Robot Simulator"
#WORKING_DIR=$(rosstack find mtconnect)/../simulator
#COMMAND1="ruby robot.rb"
#TERMINAL4_CMD="bash -c \" cd $WORKING_DIR; echo $TERMINAL4_NAME; $COMMAND1; exec bash\""

# Running each agent/simulator in its own terminal
#gnome-terminal --tab --title="AGENT1" -e "bash -c \" cd $WORKING_DIR; echo Agent 1; $COMMAND1; exec bash\""
#gnome-terminal --tab --title=$TERMINAL1_NAME -e "$TERMINAL1_CMD" --tab --title=$TERMINAL2_NAME -e "$TERMINAL2_CMD" \
#--tab --title=$TERMINAL3_NAME -e "$TERMINAL3_CMD" --tab --title=$TERMINAL4_NAME -e "$TERMINAL4_CMD"
gnome-terminal --tab --title=$TERMINAL1_NAME -e "$TERMINAL1_CMD" --tab --title=$TERMINAL2_NAME -e "$TERMINAL2_CMD" \
--tab --title=$TERMINAL3_NAME -e "$TERMINAL3_CMD"
