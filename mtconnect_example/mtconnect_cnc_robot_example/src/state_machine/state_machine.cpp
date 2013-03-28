/*
 * state_machine.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: ros developer 
 */

#include <mtconnect_cnc_robot_example/state_machine/state_machine.h>

// defaults
static const std::string PARAM_ARM_GROUP = "arm_group";
static const std::string PARAM_UNLOAD_PICKUP_GOAL = "unload_pickup_goal";
static const std::string PARAM_UNLOAD_PLACE_GOAL = "unload_place_goal";
static const std::string PARAM_LOAD_PICKUP_GOAL = "load_pickup_goal";
static const std::string PARAM_LOAD_PLACE_GOAL = "load_place_goal";
static const std::string PARAM_JOINT_HOME_POSITION = "/joint_home_position";
static const std::string PARAM_JOINT_WAIT_POSITION = "/joint_wait_position";
static const std::string PARAM_TRAJ_ARBITRARY_MOVE = "/traj_arbitrary";
static const std::string PARAM_TRAJ_APPROACH_CNC = "/traj_approach_cnc";
static const std::string PARAM_TRAJ_ENTER_CNC = "/traj_enter_cnc";
static const std::string PARAM_TRAJ_MOVE_TO_CHUCK = "traj_move_to_chuck";
static const std::string PARAM_TRAJ_RETREAT_FROM_CHUCK = "/traj_retreat_from_chuck";
static const std::string PARAM_TRAJ_EXIT_CNC = "/traj_exit_cnc";

static const std::string DEFAULT_MOVE_ARM_ACTION = "move_arm_action";
static const std::string DEFAULT_PICKUP_ACTION = "pickup_action_service";
static const std::string DEFAULT_PLACE_ACTION = "place_action_service";
static const std::string DEFAULT_GRASP_ACTION = "grasp_action_service";
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
static const double DURATION_LOOP_PAUSE = 0.5f;
static const double DURATION_TIMER_INTERVAL = 4.0f;
static const double DURATION_WAIT_SERVER = 2.0f;
static const double DURATION_PLANNING_TIME = 5.0f;
static const double DURATION_WAIT_RESULT = 40.0f;
static const double DURATION_PATH_COMPLETION = 2.0f;
static const int MAX_WAIT_ATTEMPTS = 40;

using namespace mtconnect_cnc_robot_example::state_machine;

StateMachine::StateMachine()
{
	// TODO Auto-generated constructor stub

}

StateMachine::~StateMachine()
{
	// TODO Auto-generated destructor stub
}

bool StateMachine::fetch_parameters(std::string name_space)
{
	ros::NodeHandle ph("~");
	return ph.getParam(PARAM_ARM_GROUP,arm_group_) &&
			material_load_pickup_goal_.fetchParameters(PARAM_LOAD_PICKUP_GOAL) &&
			material_unload_place_goal_.fetchParameters(PARAM_UNLOAD_PLACE_GOAL) &&
			joint_home_pos_.fetchParameters(PARAM_JOINT_HOME_POSITION) &&
			joint_wait_pos_.fetchParameters(PARAM_JOINT_WAIT_POSITION) &&
			traj_approach_cnc_.fetchParameters(PARAM_TRAJ_APPROACH_CNC) &&
			traj_enter_cnc_.fetchParameters(PARAM_TRAJ_ENTER_CNC) &&
			traj_move_to_chuck_.fetchParameters(PARAM_TRAJ_MOVE_TO_CHUCK) &&
			traj_retreat_from_chuck_.fetchParameters(PARAM_TRAJ_RETREAT_FROM_CHUCK) &&
			traj_exit_cnc_.fetchParameters(PARAM_TRAJ_EXIT_CNC) &&
			traj_arbitrary_move_.fetchParameters(PARAM_TRAJ_ARBITRARY_MOVE);
	return true;
}
bool StateMachine::setup()
{
	using namespace industrial_msgs;

	ros::NodeHandle nh;

	if(fetch_parameters())
	{
		ROS_INFO_STREAM("Read all parameters successfully");
	}
	else
	{
		ROS_ERROR_STREAM("Failed to read parameters, exiting");
		return false;
	}

	// initializing move arm client
	if(!MoveArmActionClient::setup())
	{
		ROS_ERROR_STREAM("Failed to initialize MoveArmActionClient");
		return false;
	}

	// initializing service servers
	material_load_server_ptr_ = MaterialLoadServerPtr(new MaterialLoadServer(nh,DEFAULT_MATERIAL_LOAD_ACTION,
			boost::bind(&StateMachine::material_load_goalcb,this,_1),false));

	material_unload_server_ptr_ = MaterialUnloadServerPtr(new  MaterialUnloadServer(nh,DEFAULT_MATERIAL_UNLOAD_ACTION,
					boost::bind(&StateMachine::material_unload_goalcb,this,_1),false));

	// initializing service clients
	move_pickup_client_ptr_ = MovePickupClientPtr(new MovePickupClient(DEFAULT_PICKUP_ACTION,true));
	move_place_client_ptr_ = MovePlaceClientPtr(new MovePlaceClient(DEFAULT_PLACE_ACTION,true));
	open_door_client_ptr_ =CncOpenDoorClientPtr(new CncOpenDoorClient(DEFAULT_CNC_OPEN_DOOR_ACTION,true));
	close_door_client_ptr_ =CncCloseDoorClientPtr(new CncCloseDoorClient(DEFAULT_CNC_CLOSE_DOOR_ACTION,true));
	open_chuck_client_ptr_ =CncOpenChuckClientPtr(new CncOpenChuckClient(DEFAULT_CNC_OPEN_CHUCK_ACTION,true));
	close_chuck_client_ptr_ =CncCloseChuckClientPtr(new CncCloseChuckClient(DEFAULT_CNC_CLOSE_CHUCK_ACTION,true));
	grasp_action_client_ptr_ = GraspActionClientPtr(new GraspActionClient(DEFAULT_GRASP_ACTION,true));

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
	robot_topics_timer_ = nh.createTimer(ros::Duration(DURATION_TIMER_INTERVAL),
			&StateMachine::publish_robot_topics_timercb,this,false,false);

	return true;
}

bool StateMachine::run_task_sequence(std::vector<int> &task_sequence,int &fault_state)
{
	return true;
}

void StateMachine::run_material_load_sequence()
{
	int fault_state = states::EMPTY;
	if(run_task_sequence(material_load_sequence_,fault_state))
	{
		set_active_state(states::MATERIAL_LOAD_COMPLETED);
	}
	else
	{
		// return failure
		MaterialLoadServer::Result res;
		res.load_state= "Failed";
		material_load_server_ptr_->setAborted(res);
		ROS_INFO_STREAM("MATERIAL LOAD request failed");
		set_active_state(fault_state);
	}
}

void StateMachine::run_material_unload_sequence()
{
	int fault_state = states::EMPTY;
	if(run_task_sequence(material_unload_sequence_,fault_state))
	{
		set_active_state(states::MATERIAL_UNLOAD_COMPLETED);
	}
	else
	{
		MaterialUnloadServer::Result res;
		res.unload_state= "Failed";
		material_unload_server_ptr_->setAborted(res);
		ROS_INFO_STREAM("MATERIAL UNLOAD request failed");
		set_active_state(fault_state);
	}
}

// transition actions
bool StateMachine::on_startup()
{
	return setup();

}

bool StateMachine::on_ready()
{
	return true;
}

bool StateMachine::on_robot_reset()
{
	return true;
}

bool StateMachine::on_material_load_started()
{
	action_execution_thread_ = boost::thread(&StateMachine::run_material_load_sequence,this);
	return true;
}

bool StateMachine::on_material_load_completed()
{
	MaterialLoadServer::Result res;
	res.load_state= "Succeeded";
	material_load_server_ptr_->setSucceeded(res);
	ROS_INFO_STREAM("MATERIAL LOAD request succeeded");

	return true;
}

bool StateMachine::on_material_unload_started()
{
	action_execution_thread_ = boost::thread(&StateMachine::run_material_unload_sequence,this);
	return true;
}

bool StateMachine::on_material_unload_completed()
{
	MaterialUnloadServer::Result res;
	res.unload_state= "Succeeded";
	material_unload_server_ptr_->setSucceeded(res);
	ROS_INFO_STREAM("MATERIAL UNLOAD request succeeded");
	return true;
}

bool StateMachine::on_cnc_reset()
{
	return true;
}

bool StateMachine::on_peripherals_reset()
{
	return true;
}

bool StateMachine::on_robot_fault()
{
	return true;
}

bool StateMachine::on_cnc_fault()
{
	return true;
}

bool StateMachine::on_gripper_fault()
{
	return true;
}

// callbacks
void StateMachine::material_load_goalcb(const MaterialLoadServer::GoalConstPtr &gh)
{
	if(get_active_state()== states::READY)
	{
		set_active_state(states::MATERIAL_LOAD_STARTED);
	}
}

void StateMachine::material_unload_goalcb(const MaterialUnloadServer::GoalConstPtr &gh)
{
	if(get_active_state()== states::READY)
	{
		set_active_state(states::MATERIAL_UNLOAD_STARTED);
	}
}

void StateMachine::publish_robot_topics_timercb(const ros::TimerEvent &evnt)
{
	// updating header time stamps
	robot_state_msg_.header.stamp = ros::Time::now();
	robot_spindle_msg_.header.stamp = ros::Time::now();

	// publishing
	robot_states_pub_.publish(robot_state_msg_);
	robot_spindle_pub_.publish(robot_spindle_msg_);
}

// move arm method
bool StateMachine::moveArm(move_arm_utils::JointStateInfo &joint_info)
{
	// setting goal joint constraints
	move_arm_goal_.motion_plan_request.goal_constraints.joint_constraints.clear();
	joint_info.toJointConstraints(DEFAULT_JOINT_ERROR_TOLERANCE,DEFAULT_JOINT_ERROR_TOLERANCE,
			move_arm_goal_.motion_plan_request.goal_constraints.joint_constraints);

	// sending goal
	move_arm_client_ptr_->sendGoal(move_arm_goal_,move_arm_done_cb_);
	return true;
}

