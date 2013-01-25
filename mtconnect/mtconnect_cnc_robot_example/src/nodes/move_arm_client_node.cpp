/*
 * move_arm_client_node.cpp
 *
 *  Created on: Jan 16, 2013
 */

#include <mtconnect_cnc_robot_example/move_arm_action_clients/MoveArmActionClient.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"move_arm_client_node");
	ros::NodeHandle nh;

	MoveArmActionClient client;
	client.run();

	return 0;
}
