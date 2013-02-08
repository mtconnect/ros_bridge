
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

#include <mtconnect_msgs/CncStatus.h>
#include <mtconnect_msgs/CloseChuckAction.h>
#include <mtconnect_msgs/OpenChuckAction.h>
#include <mtconnect_msgs/CloseDoorAction.h>
#include <mtconnect_msgs/OpenDoorAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

// alias
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseChuckAction> CncOpenDoorClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseChuckAction> CncCloseDoorClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseChuckAction> CncOpenChuckClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseChuckAction> CncCloseChuckClient;
typedef boost::shared_ptr<CncOpenDoorClient> CncOpenDoorClientPtr;
typedef boost::shared_ptr<CncCloseDoorClient> CncCloseDoorClientPtr;
typedef boost::shared_ptr<CncOpenChuckClient> CncOpenChuckClientPtr;
typedef boost::shared_ptr<CncCloseChuckClient> CncCloseChuckClienPtr;

// defaults
static const std::string CNC_OPEN_DOOR_SERVICE = "cnc_service";
static const double DURATION_WAIT_SERVER = 4.0f;
static const int MAX_WAIT_ATTEMPTS = 10;

class SimpleTaskManager
{
public:
	SimpleTaskManager()
	{

	}

	~SimpleTaskManager()
	{

	}

	void run()
	{
		if(!setup())
		{
			return;
		}
	}

protected:

	bool setup()
	{
		ros::NodeHandle nh;
		int wait_attempts = 0;

		// initializing service client
		open_door_client_ptr =CncOpenDoorClientPtr(new CncOpenDoorClient(CNC_OPEN_DOOR_SERVICE,true));

		// waiting for service service
		while(!open_door_client_ptr->waitForServer(ros::Duration(DURATION_WAIT_SERVER)))
		{
			if(wait_attempts++ > MAX_WAIT_ATTEMPTS)
			{
				return false;
			}
		}
	}

	CncOpenDoorClientPtr open_door_client_ptr;

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"simple_task_manager");

	SimpleTaskManager task_manager;
	task_manager.run();
	return 0;
}
