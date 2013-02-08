
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
#include <mtconnect_msgs/MaterialLoadAction.h>
#include <mtconnect_msgs/MaterialUnloadAction.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <mtconnect_cnc_robot_example/utilities/utilities.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

// aliases
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> MovePickupClient;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> MovePlaceClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseChuckAction> CncOpenDoorClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseChuckAction> CncCloseDoorClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseChuckAction> CncOpenChuckClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseChuckAction> CncCloseChuckClient;
typedef actionlib::SimpleActionServer<mtconnect_msgs::MaterialLoadAction> MaterialLoadServer;
typedef actionlib::SimpleActionServer<mtconnect_msgs::MaterialUnloadAction> MaterialUnloadServer;
typedef boost::shared_ptr<MoveArmClient> MoveArmClientPtr;
typedef boost::shared_ptr<MovePickupClient> MovePickupClientPtr;
typedef boost::shared_ptr<MovePlaceClient> MovePlaceClientPtr;
typedef boost::shared_ptr<CncOpenDoorClient> CncOpenDoorClientPtr;
typedef boost::shared_ptr<CncCloseDoorClient> CncCloseDoorClientPtr;
typedef boost::shared_ptr<CncOpenChuckClient> CncOpenChuckClientPtr;
typedef boost::shared_ptr<CncCloseChuckClient> CncCloseChuckClientPtr;
typedef boost::shared_ptr<MaterialLoadServer> MaterialLoadServerPtr;
typedef boost::shared_ptr<MaterialUnloadServer> MaterialUnloadServerPtr;

// defaults
static const std::string DEFAULT_MOVE_ARM_ACTION = "move_arm_action";
static const std::string DEFAULT_PICKUP_ACTION = "pickup_action_service";
static const std::string DEFAULT_PLACE_ACTION = "place_action_service";
static const std::string DEFAULT_MATERIAL_LOAD_ACTION= "material_load_action";
static const std::string DEFAULT_MATERIAL_UNLOAD_ACTION= "material_unload_action";
static const std::string DEFAULT_CNC_OPEN_DOOR_ACTION = "cnc_open_door_action";
static const std::string DEFAULT_CNC_CLOSE_DOOR_ACTION = "cnc_close_door_action";
static const std::string DEFAULT_CNC_OPEN_CHUCK_ACTION = "cnc_open_chuck_action";
static const std::string DEFAULT_CNC_CLOSE_CHUCK_ACTION = "cnc_close_chuck_action";
static const double DEFAULT_JOINT_ERROR_TOLERANCE = 0.01f; // radians
static const double DURATION_LOOP_PAUSE = 2.0f;
static const double DURATION_WAIT_SERVER = 2.0f;
static const int MAX_WAIT_ATTEMPTS = 10;


class SimpleMaterialHandlingServer
{
public:
	SimpleMaterialHandlingServer()
	{

	}

	~SimpleMaterialHandlingServer()
	{

	}

	void run()
	{
		if(!setup())
		{
			return;
		}

		ros::AsyncSpinner spinner(4);
		spinner.start();

		// starting material handling servers
		material_load_server_ptr_->start();
		material_unload_server_ptr_->start();

		while(ros::ok())
		{
			ros::Duration(DURATION_LOOP_PAUSE).sleep();
		}
	}

protected:

	bool fetchParameters(std::string ns = "")
	{
		return pickup_goal_.fetchParameters() && place_goal_.fetchParameters();
	}

	bool setup()
	{
		ros::NodeHandle nh;
		int wait_attempts = 0;

		if(!fetchParameters())
		{
			ROS_ERROR_STREAM("Failed to read parameters, exiting");
			return false;
		}

		// initializing service servers
		material_load_server_ptr_ = MaterialLoadServerPtr(new MaterialLoadServer(nh,DEFAULT_MATERIAL_LOAD_ACTION,
				boost::bind(&SimpleMaterialHandlingServer::materialLoadGoalCallback,this,_1),false));

		material_unload_server_ptr_ = MaterialUnloadServerPtr(new  MaterialUnloadServer(nh,DEFAULT_MATERIAL_UNLOAD_ACTION,
						boost::bind(&SimpleMaterialHandlingServer::materialUnloadGoalCallback,this,_1),false));

		// initializing service clients
		move_arm_client_ptr_ = MoveArmClientPtr(new MoveArmClient(DEFAULT_MOVE_ARM_ACTION,true));
		arm_pickup_client_ptr_ = MovePickupClientPtr(new MovePickupClient(DEFAULT_PICKUP_ACTION,true));
		arm_place_client_ptr_ = MovePlaceClientPtr(new MovePlaceClient(DEFAULT_PLACE_ACTION,true));
		open_door_client_ptr_ =CncOpenDoorClientPtr(new CncOpenDoorClient(DEFAULT_CNC_OPEN_DOOR_ACTION,true));
		close_door_client_ptr_ =CncCloseDoorClientPtr(new CncCloseDoorClient(DEFAULT_CNC_CLOSE_DOOR_ACTION,true));
		open_chuck_client_ptr_ =CncOpenChuckClientPtr(new CncOpenDoorClient(DEFAULT_CNC_OPEN_CHUCK_ACTION,true));
		close_chuck_client_ptr_ =CncCloseChuckClientPtr(new CncCloseChuckClient(DEFAULT_CNC_CLOSE_CHUCK_ACTION,true));

		// waiting for service service
		while(	(!arm_pickup_client_ptr_->isServerConnected() && !arm_pickup_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!arm_place_client_ptr_->isServerConnected() && !arm_place_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!open_door_client_ptr_->isServerConnected() && !open_door_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!close_door_client_ptr_->isServerConnected() && !close_door_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!open_chuck_client_ptr_->isServerConnected() && !open_chuck_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!close_chuck_client_ptr_->isServerConnected() && !close_chuck_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) )
		{
			if(wait_attempts++ > MAX_WAIT_ATTEMPTS)
			{
				ROS_ERROR_STREAM("One or more action servers were not found, exiting");
				return false;
			}
			ROS_WARN_STREAM("Waiting for action servers");
		}

		ROS_INFO_STREAM("Found all action servers");
		return true;
	}

	void materialLoadGoalCallback(const MaterialLoadServer::GoalConstPtr &gh)
	{

	}

	void materialUnloadGoalCallback(const MaterialUnloadServer::GoalConstPtr &gh)
	{

	}


protected:

	// action servers
	MaterialLoadServerPtr material_load_server_ptr_;
	MaterialUnloadServerPtr material_unload_server_ptr_;

	// action clients
	MoveArmClientPtr move_arm_client_ptr_;
	MovePickupClientPtr arm_pickup_client_ptr_;
	MovePlaceClientPtr arm_place_client_ptr_;
	CncOpenDoorClientPtr open_door_client_ptr_;
	CncCloseDoorClientPtr close_door_client_ptr_;
	CncOpenChuckClientPtr open_chuck_client_ptr_;
	CncCloseChuckClientPtr close_chuck_client_ptr_;

	// pick place info members
	move_arm_utils::PickupGoalInfo pickup_goal_;
	move_arm_utils::PlaceGoalInfo place_goal_;

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"material_hanlding_server_test");

	SimpleMaterialHandlingServer material_handling_server;
	material_handling_server.run();
	return 0;
}
