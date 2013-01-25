/*
 * MovePickPlaceServer.cpp
 *
 *  Created on: Jan 25, 2013
 */

#include <mtconnect_cnc_robot_example/move_arm_action_clients/MovePickPlaceServer.h>
#include <boost/bind.hpp>

MovePickPlaceServer::MovePickPlaceServer() :
	MoveArmActionClient()
{
	// TODO Auto-generated constructor stub

}

MovePickPlaceServer::~MovePickPlaceServer()
{
	// TODO Auto-generated destructor stub
}

void MovePickPlaceServer::run()
{

}

bool MovePickPlaceServer::fetchParameters(std::string name_space)
{
	return true;
}

bool MovePickPlaceServer::setup()
{
	if(!MoveArmActionClient::setup())
	{
		return false;
	}

	// ros handle
	ros::NodeHandle nh;

	// setting up pickup server
	arm_pickup_server_ptr_  = MoveArmPickupServerPtr(new MoveArmPickupServer(nh,DEFAULT_PICKUP_ACTION,
			boost::bind(&MovePickPlaceServer::pickupGoalCallback,this, _1),
			boost::bind(&MovePickPlaceServer::pickupCancelCallback,this,_1),
			false));

	// setting up place server
	arm_place_server_ptr_  = MoveArmPlaceServerPtr(new MoveArmPlaceServer(nh,DEFAULT_PLACE_ACTION,
			boost::bind(&MovePickPlaceServer::placeGoalCallback,this, _1),
			boost::bind(&MovePickPlaceServer::placeCancelCallback,this,_1),
			false));

	// setting up grasp action client
	grasp_action_client_ptr_ = GraspActionClientPtr(new GraspActionClient(DEFAULT_GRASP_ACTION,true));

	return true;
}

bool MovePickPlaceServer::moveArmThroughPickSequence(object_manipulation_msgs::PickupGoal &pickup_goal)
{
	return true;
}

bool MovePickPlaceServer::moveArmThroughPlaceSequence(object_manipulation_msgs::PlaceGoal &place_goal)
{
	return true;
}

void MovePickPlaceServer::pickupGoalCallback(PickupGoalHandle goal)
{

}

void MovePickPlaceServer::pickupCancelCallback(PickupGoalHandle goal)
{

}

void MovePickPlaceServer::placeGoalCallback(PlaceGoalHandle goal)
{

}

void MovePickPlaceServer::placeCancelCallback(PlaceGoalHandle goal)
{

}

void MovePickPlaceServer::createPickupMoveSequence(const object_manipulation_msgs::PickupGoal &goal,
		object_manipulator::GraspExecutionInfo &seq)
{

}

void MovePickPlaceServer::createPlaceMoveSequence(const object_manipulation_msgs::PlaceGoal &goal,
		object_manipulator::PlaceExecutionInfo &seq)
{

}
