
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

#include <mtconnect_cnc_robot_example/move_arm_action_clients/MovePickPlaceServer.h>

using namespace mtconnect_cnc_robot_example;

class MovePickPlaceTest: public MovePickPlaceServer
{
public:
	MovePickPlaceTest():
		MovePickPlaceServer()
	{

	}

	~MovePickPlaceTest()
	{

	}


	virtual void run()
	{
		if(!setup())
		{
			return;
		}

		ros::AsyncSpinner spinner(2);
		spinner.start();

		while(ros::ok())
		{
			if(moveArmThroughPickSequence(pickup_goal_))
			{
				ROS_INFO_STREAM("Pickup move completed");
			}
			else
			{
				ROS_ERROR_STREAM("Pickup move failed, exiting");
				break;
			}

			if(moveArmThroughPlaceSequence(place_goal_))
			{
				ROS_INFO_STREAM("Place move completed");
			}
			else
			{
				ROS_ERROR_STREAM("Place move failed, exiting");
				break;
			}
		}
	}

	virtual bool fetchParameters()
	{
		// fetching pick and place parameters for protected pick place members (only needed if not servicing client requests)
		if(MovePickPlaceServer::fetchParameters() && pickup_goal_.fetchParameters() && place_goal_.fetchParameters())
		{
			ROS_INFO_STREAM("Successfully read arm and pick place info parameters");
			return true;
		}
		else
		{
			ROS_ERROR_STREAM("Failed to read arm and pick place info parameters");
			return false;
		}
	}

public:

	PickupGoalInfo pickup_goal_;
	PlaceGoalInfo place_goal_;
};

int main(int argc,char** argv)
{


	ros::init(argc,argv,"move_pick_place_test");
	MoveArmActionClientPtr arm_client_ptr = MoveArmActionClientPtr(new MovePickPlaceTest());

	// run through pick place motions
	arm_client_ptr->run();

	return 0;
}
