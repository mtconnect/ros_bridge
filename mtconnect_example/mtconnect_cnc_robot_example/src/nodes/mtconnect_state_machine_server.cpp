/*
 * mtconnect_state_machine_server.cpp
 *
 *  Created on: Mar 27, 2013
 *      Author: ros developer 
 */

#include <mtconnect_cnc_robot_example/state_machine/state_machine.h>
using namespace mtconnect_cnc_robot_example::state_machine;

int main(int argc,char** argv)
{
	ros::init(argc,argv,"mtconnect_state_machine_server");
	ros::NodeHandle nh;

	StateMachineInterface::Ptr s(new StateMachine());
	s->run();

	return 0;
}
