/*
 * move_pick_place_server_node.cpp
 *
 *  Created on: Jan 25, 2013
 */

#include <mtconnect_cnc_robot_example/move_arm_action_clients/MovePickPlaceServer.h>

int main(int argc,char** argv)
{
	using namespace mtconnect_cnc_robot_example;

	ros::init(argc,argv,"move_pick_place_server_node");
	MoveArmActionClientPtr arm_client_ptr = MoveArmActionClientPtr(new MovePickPlaceServer());
	arm_client_ptr->run();

	return 0;
}
