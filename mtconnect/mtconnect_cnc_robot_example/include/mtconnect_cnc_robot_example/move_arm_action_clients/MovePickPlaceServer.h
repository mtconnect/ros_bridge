
/*
 * Copyright 2013 Southwest Research Institute

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#ifndef MOVEPICKPLACESERVER_H_
#define MOVEPICKPLACESERVER_H_

#include "MoveArmActionClient.h"
#include <object_manipulator/grasp_execution/approach_lift_grasp.h>
#include <object_manipulator/place_execution/descend_retreat_place.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <actionlib/server/simple_action_server.h>

namespace mtconnect_cnc_robot_example
{
// aliases
typedef actionlib::ActionServer<object_manipulation_msgs::PickupAction> MoveArmPickupServer;
typedef actionlib::ActionServer<object_manipulation_msgs::PlaceAction> MoveArmPlaceServer;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;
typedef object_manipulation_msgs::GraspHandPostureExecutionGoal GraspGoal;
typedef boost::shared_ptr<MoveArmPickupServer> MoveArmPickupServerPtr;
typedef boost::shared_ptr<MoveArmPlaceServer> MoveArmPlaceServerPtr;
typedef boost::shared_ptr<GraspActionClient> GraspActionClientPtr;
typedef MoveArmPickupServer::GoalHandle PickupGoalHandle;
typedef MoveArmPlaceServer::GoalHandle PlaceGoalHandle;

// defaults
static const std::string DEFAULT_PICKUP_ACTION = "pickup_action_service";
static const std::string DEFAULT_PLACE_ACTION = "place_action_service";
static const std::string DEFAULT_GRASP_ACTION = "grasp_action_service";

class MovePickPlaceServer: public MoveArmActionClient
{
public:
	MovePickPlaceServer();
	virtual ~MovePickPlaceServer();

	virtual void run();
	virtual bool fetchParameters(std::string nameSpace = "");

	virtual bool moveArmThroughPickSequence(object_manipulation_msgs::PickupGoal &pickup_goal);
	virtual bool moveArmThroughPlaceSequence(object_manipulation_msgs::PlaceGoal &place_goal);

protected:

	virtual bool setup();

protected:

	virtual void pickupGoalCallback(PickupGoalHandle goal);
	virtual void pickupCancelCallback(PickupGoalHandle goal);
	virtual void placeGoalCallback(PlaceGoalHandle goal);
	virtual void placeCancelCallback(PlaceGoalHandle goal);

	bool createPickupMoveSequence(const object_manipulation_msgs::PickupGoal &goal
			,geometry_msgs::PoseArray &pickup_pose_sequence);
	bool createPlaceMoveSequence(const object_manipulation_msgs::PlaceGoal &goal
			,geometry_msgs::PoseArray &place_pose_sequence);

protected:

	// action servers
	MoveArmPickupServerPtr arm_pickup_server_ptr_;
	MoveArmPlaceServerPtr arm_place_server_ptr_;

	// action clients
	GraspActionClientPtr grasp_action_client_ptr_;

	// pick place move info
	PickupGoalInfo pickup_goal_;
	PlaceGoalInfo place_goal_;

};

}
#endif /* MOVEPICKPLACESERVER_H_ */
