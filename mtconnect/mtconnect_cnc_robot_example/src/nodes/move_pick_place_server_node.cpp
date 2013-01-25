/*
 * move_pick_place_server_node.cpp
 *
 *  Created on: Jan 25, 2013
 */

#include <mtconnect_cnc_robot_example/move_arm_action_clients/MovePickPlaceServer.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"move_pick_place_server_node");
	MovePickPlaceServer move_pick_place_server;
	move_pick_place_server.run();

	return 0;
}
