
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

//#include <mtconnect_msgs/CncStatus.h>
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
#include <boost/assign/list_inserter.hpp>
#include <boost/assign/list_of.hpp>
#include <mtconnect_msgs/RobotSpindle.h>
#include <mtconnect_msgs/RobotStates.h>
#include <boost/thread.hpp>

// aliases
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> MovePickupClient;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> MovePlaceClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::OpenDoorAction> CncOpenDoorClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseDoorAction> CncCloseDoorClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::OpenChuckAction> CncOpenChuckClient;
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
typedef std::vector<int> MaterialHandlingSequence;

// defaults
static const std::string PARAM_ARM_GROUP = "arm_group";
static const std::string PARAM_UNLOAD_PICKUP_GOAL = "unload_pickup_goal";
static const std::string PARAM_UNLOAD_PLACE_GOAL = "unload_place_goal";
static const std::string PARAM_LOAD_PICKUP_GOAL = "load_pickup_goal";
static const std::string PARAM_LOAD_PLACE_GOAL = "load_place_goal";
static const std::string PARAM_JOINT_HOME_POSITION = "/joint_home_position";
static const std::string PARAM_JOINT_WAIT_POSITION = "/joint_wait_position";
static const std::string DEFAULT_MOVE_ARM_ACTION = "move_arm_action";
static const std::string DEFAULT_PICKUP_ACTION = "pickup_action_service";
static const std::string DEFAULT_PLACE_ACTION = "place_action_service";
static const std::string DEFAULT_MATERIAL_LOAD_ACTION= "material_load_action";
static const std::string DEFAULT_MATERIAL_UNLOAD_ACTION= "material_unload_action";
static const std::string DEFAULT_CNC_OPEN_DOOR_ACTION = "cnc_open_door_action";
static const std::string DEFAULT_CNC_CLOSE_DOOR_ACTION = "cnc_close_door_action";
static const std::string DEFAULT_CNC_OPEN_CHUCK_ACTION = "cnc_open_chuck_action";
static const std::string DEFAULT_CNC_CLOSE_CHUCK_ACTION = "cnc_close_chuck_action";
static const std::string DEFAULT_ROBOT_STATES_TOPIC = "robot_states";
static const std::string DEFAULT_ROBOT_SPINDLE_TOPIC = "robot_spindle";
static const std::string CNC_ACTION_ACTIVE_FLAG = "ACTIVE";
static const double DEFAULT_JOINT_ERROR_TOLERANCE = 0.01f; // radians
static const int DEFAULT_PATH_PLANNING_ATTEMPTS = 2;
static const std::string DEFAULT_PATH_PLANNER = "/ompl_planning/plan_kinematic_path";
static const double DURATION_LOOP_PAUSE = 2.0f;
static const double DURATION_TIMER_INTERVAL = 4.0f;
static const double DURATION_WAIT_SERVER = 2.0f;
static const double DURATION_PLANNING_TIME = 5.0f;
static const double DURATION_WAIT_RESULT = 40.0f;
static const int MAX_WAIT_ATTEMPTS = 40;

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

		// moving the arm home

		while(ros::ok())
		{
			ros::Duration(DURATION_LOOP_PAUSE).sleep();
		}
	}

public:

	MaterialHandlingSequence material_load_sequence_;
	MaterialHandlingSequence material_unload_sequence_;

protected:

	bool fetchParameters(std::string ns = "")
	{
		ros::NodeHandle ph("~");
		return ph.getParam(PARAM_ARM_GROUP,arm_group_) &&
				material_load_pickup_goal_.fetchParameters(PARAM_LOAD_PICKUP_GOAL) && material_load_place_goal_.fetchParameters(PARAM_LOAD_PLACE_GOAL) &&
				material_unload_pickup_goal_.fetchParameters(PARAM_UNLOAD_PICKUP_GOAL) && material_unload_place_goal_.fetchParameters(PARAM_UNLOAD_PLACE_GOAL) &&
				joint_home_pos_.fetchParameters(PARAM_JOINT_HOME_POSITION) && joint_wait_pos_.fetchParameters(PARAM_JOINT_WAIT_POSITION);
	}

	bool setup()
	{
		using namespace boost::assign;
		using namespace industrial_msgs;
		typedef mtconnect_msgs::MaterialHandlingFeedback Feedback;

		ros::NodeHandle nh;
		int wait_attempts = 0;

		if(fetchParameters())
		{
			ROS_INFO_STREAM("Read all parameters successfully");
		}
		else
		{
			ROS_ERROR_STREAM("Failed to read parameters, exiting");
			return false;
		}

		// initializing task sequence lists
		material_load_sequence_ = list_of((int)Feedback::OPENNING_DOOR)((int)Feedback::OPENNING_CHUCK)
				((int)Feedback::TRANSPORTING_MATERIAL)((int)Feedback::PLACING_MATERIAL)
				((int)Feedback::CLEARING_WORK_AREA)((int)Feedback::CLOSING_CHUCK)
				((int)Feedback::CLOSING_DOOR);

		material_unload_sequence_ = list_of((int)Feedback::OPENNING_DOOR)((int)Feedback::OPENNING_CHUCK)
				((int)Feedback::RETRIEVING_WORKPIECE)((int)Feedback::TRANSPORTING_WORKPIECE)
				((int)Feedback::CLOSING_CHUCK)((int)Feedback::CLOSING_DOOR);

		// setting up move arm goal
		move_arm_goal_.motion_plan_request.group_name = arm_group_;
		move_arm_goal_.motion_plan_request.num_planning_attempts = 2;
		move_arm_goal_.planner_service_name = DEFAULT_PATH_PLANNER;
		move_arm_goal_.motion_plan_request.allowed_planning_time = ros::Duration(DURATION_PLANNING_TIME);
		move_arm_goal_.motion_plan_request.planner_id = "";

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
		open_chuck_client_ptr_ =CncOpenChuckClientPtr(new CncOpenChuckClient(DEFAULT_CNC_OPEN_CHUCK_ACTION,true));
		close_chuck_client_ptr_ =CncCloseChuckClientPtr(new CncCloseChuckClient(DEFAULT_CNC_CLOSE_CHUCK_ACTION,true));

		// initializing publishers
		robot_states_pub_ = nh.advertise<mtconnect_msgs::RobotStates>(DEFAULT_ROBOT_STATES_TOPIC,1);
		robot_spindle_pub_ = nh.advertise<mtconnect_msgs::RobotSpindle>(DEFAULT_ROBOT_SPINDLE_TOPIC,1);

		// initializing mtconnect robot messages
		robot_state_msg_.avail.val = TriState::ENABLED;
		robot_state_msg_.mode.val = RobotMode::AUTO;
		robot_state_msg_.rexec.val = TriState::HIGH;
		robot_spindle_msg_.c_unclamp.val = TriState::HIGH;
		robot_spindle_msg_.s_inter.val = TriState::HIGH;

		// initializing timers
		publish_timer_ = nh.createTimer(ros::Duration(DURATION_TIMER_INTERVAL),
				&SimpleMaterialHandlingServer::publishTimerCallback,this,false,false);


		// waiting for robot related service servers
		while(	ros::ok() && (
				(!move_arm_client_ptr_->isServerConnected() && !move_arm_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!arm_pickup_client_ptr_->isServerConnected() && !arm_pickup_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!arm_place_client_ptr_->isServerConnected() && !arm_place_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!open_door_client_ptr_->isServerConnected() && !open_door_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!close_door_client_ptr_->isServerConnected() && !close_door_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!open_chuck_client_ptr_->isServerConnected() && !open_chuck_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
				(!close_chuck_client_ptr_->isServerConnected() && !close_chuck_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER)))
				))
		{
			if(wait_attempts++ > MAX_WAIT_ATTEMPTS)
			{
				ROS_ERROR_STREAM("One or more action cnc/robot servers were not found, exiting");
				return false;
			}
			ROS_WARN_STREAM("Waiting for cnc/robot action servers");
			ros::Duration(DURATION_WAIT_SERVER).sleep();
		}

		publish_timer_.start();

		ROS_INFO_STREAM("Found all action servers");
		return true;
	}

	void materialLoadGoalCallback(const MaterialLoadServer::GoalConstPtr &gh)
	{
		// declaring result
		MaterialLoadServer::Result res;

		ROS_INFO_STREAM("Received Material Load request");
		material_load_server_ptr_->acceptNewGoal();
		if(executeMaterialHandlingTasks(material_load_sequence_))
		{
			res.load_state = "Succeeded";
			material_load_server_ptr_->setSucceeded(res);
		}
		else
		{
			res.load_state = "Failed";
			material_load_server_ptr_->setAborted(res);
		}
	}

	void materialUnloadGoalCallback(const MaterialUnloadServer::GoalConstPtr &gh)
	{
		// declaring result
		MaterialUnloadServer::Result res;

		material_unload_server_ptr_->acceptNewGoal();
		ROS_INFO_STREAM("Received Material Unload request");
		if(executeMaterialHandlingTasks(material_unload_sequence_))
		{
			res.unload_state= "Succeeded";
			material_unload_server_ptr_->setSucceeded(res);
		}
		else
		{
			res.unload_state = "Failed";
			material_unload_server_ptr_->setAborted(res);
		}
	}

	bool executeMaterialHandlingTasks(const MaterialHandlingSequence &seq)
	{
		// local aliases
		typedef mtconnect_msgs::MaterialHandlingFeedback Feedback;
		typedef MaterialHandlingSequence::const_iterator ConstSequenceIterator;

		// strings
		std::string task_name;

		// action messages
		mtconnect_msgs::OpenDoorGoal open_door_goal;
		mtconnect_msgs::CloseDoorGoal close_door_goal;
		mtconnect_msgs::OpenChuckGoal open_chuck_goal;
		mtconnect_msgs::CloseChuckGoal close_chuck_goal;
		open_door_goal.open_door = CNC_ACTION_ACTIVE_FLAG;
		close_door_goal.close_door = CNC_ACTION_ACTIVE_FLAG;
		open_chuck_goal.open_chuck = CNC_ACTION_ACTIVE_FLAG;
		close_chuck_goal.close_chuck = CNC_ACTION_ACTIVE_FLAG;

		for(ConstSequenceIterator i = seq.begin(); i != seq.end(); i++)
		{
			switch (*i)
			{
			case Feedback::TRANSPORTING_MATERIAL:

				task_name = "TRANSPORTING_MATERIAL";
				ROS_INFO_STREAM(task_name <<" task in progress");
				arm_pickup_client_ptr_->sendGoal(material_load_pickup_goal_);
				if(arm_pickup_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT)))
				{
					if(arm_pickup_client_ptr_->getResult()->manipulation_result.value != object_manipulation_msgs::ManipulationResult::SUCCESS)
					{
						ROS_ERROR_STREAM(task_name <<" task failed with status '"<< arm_pickup_client_ptr_->getResult()->manipulation_result.value
								<<"', exiting");
						return false;
					}

					ROS_INFO_STREAM(task_name <<" task completed");
				}
				else
				{
					ROS_ERROR_STREAM(task_name <<" task failed, exiting");
					return false;
				}
				break;

			case Feedback::TRANSPORTING_WORKPIECE:

				task_name = "TRANSPORTING_WORKPIECE";
				ROS_INFO_STREAM(task_name <<" task in progress");
				arm_place_client_ptr_->sendGoal(material_unload_place_goal_);
				if(arm_place_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT)))
				{
					if(arm_place_client_ptr_->getResult()->manipulation_result.value != object_manipulation_msgs::ManipulationResult::SUCCESS)
					{
						ROS_ERROR_STREAM(task_name <<" task failed with status '"<< arm_place_client_ptr_->getResult()->manipulation_result.value
								<<"', exiting");
						return false;
					}

					ROS_INFO_STREAM(task_name <<" task completed");
				}
				else
				{
					ROS_ERROR_STREAM(task_name <<" task failed, exiting");
					return false;
				}
				break;

			case Feedback::OPENNING_DOOR:

				task_name = "OPENNING_DOOR";
				ROS_INFO_STREAM(task_name <<" task in progress");
				open_door_client_ptr_->sendGoal(open_door_goal);
				if(open_door_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT)))
				{
					ROS_INFO_STREAM(task_name <<" task completed");
				}
				else
				{
					ROS_ERROR_STREAM(task_name <<" task failed, exiting");
					return false;
				}
				break;

			case Feedback::OPENNING_CHUCK:

				task_name = "OPENNING_CHUCK";
				ROS_INFO_STREAM(task_name <<" task in progress");
				open_chuck_client_ptr_->sendGoal(open_chuck_goal);
				if(open_chuck_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT)))
				{
					ROS_INFO_STREAM(task_name <<" task completed");
				}
				else
				{
					ROS_ERROR_STREAM(task_name <<" task failed, exiting");
					return false;
				}
				break;

				break;
			case Feedback::CLOSING_CHUCK:

				task_name = "CLOSING_CHUCK";
				ROS_INFO_STREAM(task_name <<" task in progress");
				close_chuck_client_ptr_->sendGoal(close_chuck_goal);
				if(close_chuck_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT)))
				{
					ROS_INFO_STREAM(task_name <<" task completed");
				}
				else
				{
					ROS_ERROR_STREAM(task_name <<" task failed, exiting");
					return false;
				}
				break;

			case Feedback::CLOSING_DOOR:

				task_name = "CLOSING_DOOR";
				ROS_INFO_STREAM(task_name <<" task in progress");
				close_door_client_ptr_->sendGoal(close_door_goal);
				if(close_door_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT)))
				{
					ROS_INFO_STREAM(task_name <<" task completed");
				}
				else
				{
					ROS_ERROR_STREAM(task_name <<" task failed, exiting");
					return false;
				}
				break;

			case Feedback::PLACING_MATERIAL:

				task_name = "PLACING_MATERIAL";
				ROS_INFO_STREAM(task_name <<" task in progress");
				arm_place_client_ptr_->sendGoal(material_load_place_goal_);
				if(arm_place_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT)))
				{
					if(arm_place_client_ptr_->getResult()->manipulation_result.value != object_manipulation_msgs::ManipulationResult::SUCCESS)
					{
						ROS_ERROR_STREAM(task_name <<" task failed with status '"<< arm_place_client_ptr_->getResult()->manipulation_result.value
								<<"', exiting");
						return false;
					}

					ROS_INFO_STREAM(task_name <<" task completed");
				}
				else
				{
					ROS_ERROR_STREAM(task_name <<" task failed, exiting");
					return false;
				}
				break;

			case Feedback::RETRIEVING_WORKPIECE:

				task_name = "RETRIEVING_WORKPIECE";
				ROS_INFO_STREAM(task_name <<" task in progress");
				arm_pickup_client_ptr_->sendGoal(material_unload_pickup_goal_);
				if(arm_pickup_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT)))
				{
					if(arm_pickup_client_ptr_->getResult()->manipulation_result.value != object_manipulation_msgs::ManipulationResult::SUCCESS)
					{
						ROS_ERROR_STREAM(task_name <<" task failed with status '"<< arm_pickup_client_ptr_->getResult()->manipulation_result.value
								<<"', exiting");
						return false;
					}

					ROS_INFO_STREAM(task_name <<" task completed");
				}
				else
				{
					ROS_ERROR_STREAM(task_name <<" task failed, exiting");
					return false;
				}
				break;

			case Feedback::WAITING_ON_EXTERNAL_DEVICE:
				break;
			case Feedback::CLEARING_WORK_AREA:

				task_name = "CLEARING_WORK_AREA";
				ROS_INFO_STREAM(task_name <<" task in progress");

				// setting goal joint constraints
				move_arm_goal_.motion_plan_request.goal_constraints.joint_constraints.clear();
				joint_wait_pos_.toJointConstraints(DEFAULT_JOINT_ERROR_TOLERANCE,DEFAULT_JOINT_ERROR_TOLERANCE,
						move_arm_goal_.motion_plan_request.goal_constraints.joint_constraints);

				move_arm_client_ptr_->sendGoal(move_arm_goal_);
				if(move_arm_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT)))
				{
					if(move_arm_client_ptr_->getResult()->error_code.val != arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS)
					{
						ROS_ERROR_STREAM(task_name <<" task failed, exiting");
						return false;
					}

					ROS_INFO_STREAM(task_name <<" task completed");
				}
				else
				{
					ROS_ERROR_STREAM(task_name <<" task failed, exiting");
					return false;
				}
				break;


			}
		}
		return true;
	}

	void publishTimerCallback(const ros::TimerEvent &evnt)
	{
		// updating header time stamps
		robot_state_msg_.header.stamp = ros::Time::now();
		robot_spindle_msg_.header.stamp = ros::Time::now();

		// publishing
		robot_states_pub_.publish(robot_state_msg_);
		robot_spindle_pub_.publish(robot_spindle_msg_);
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

	// topic publishers (ros bridge components wait for these topics)
	ros::Publisher robot_states_pub_;
	ros::Publisher robot_spindle_pub_;

	// robot state messages
	mtconnect_msgs::RobotStates robot_state_msg_;
	mtconnect_msgs::RobotSpindle robot_spindle_msg_;

	// timers
	ros::Timer publish_timer_;

	// arm group, pick, place and joint info members
	std::string arm_group_;
	arm_navigation_msgs::MoveArmGoal move_arm_goal_;
	move_arm_utils::PickupGoalInfo material_load_pickup_goal_;
	move_arm_utils::PlaceGoalInfo material_load_place_goal_;
	move_arm_utils::PickupGoalInfo material_unload_pickup_goal_;
	move_arm_utils::PlaceGoalInfo material_unload_place_goal_;
	move_arm_utils::JointStateInfo joint_home_pos_;
	move_arm_utils::JointStateInfo joint_wait_pos_;

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"material_hanlding_server_test");

	SimpleMaterialHandlingServer material_handling_server;
	material_handling_server.run();
	return 0;
}
