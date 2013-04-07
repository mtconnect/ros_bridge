/*
 * state_machine.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: ros developer 
 */

#include <mtconnect_cnc_robot_example/state_machine/state_machine.h>
#include <ros/topic.h>
// params
static const std::string PARAM_ARM_GROUP = "arm_group";
static const std::string PARAM_UNLOAD_PICKUP_GOAL = "unload_pickup_goal";
static const std::string PARAM_UNLOAD_PLACE_GOAL = "unload_place_goal";
static const std::string PARAM_LOAD_PICKUP_GOAL = "load_pickup_goal";
static const std::string PARAM_LOAD_PLACE_GOAL = "load_place_goal";
static const std::string PARAM_JOINT_HOME_POSITION = "joint_home_position";
static const std::string PARAM_JOINT_WAIT_POSITION = "joint_wait_position";
static const std::string PARAM_TRAJ_ARBITRARY_MOVE = "traj_arbitrary";
static const std::string PARAM_TRAJ_APPROACH_CNC = "traj_approach_cnc";
static const std::string PARAM_TRAJ_ENTER_CNC = "traj_enter_cnc";
static const std::string PARAM_TRAJ_MOVE_TO_CHUCK = "traj_move_to_chuck";
static const std::string PARAM_TRAJ_RETREAT_FROM_CHUCK = "traj_retreat_from_chuck";
static const std::string PARAM_TRAJ_EXIT_CNC = "traj_exit_cnc";
static const std::string PARAM_FORCE_ROBOT_FAULT = "force_robot_fault";
static const std::string PARAM_FORCE_CNC_FAULT = "force_cnc_fault";
static const std::string PARAM_FORCE_GRIPPER_FAULT = "force_gripper_fault";
static const std::string PARAM_FORCE_FAULT_ON_TASK = "force_fault_on_task";
static const std::string PARAM_TASK_DESCRIPTION = "task_description";
static const std::string PARAM_USE_TASK_MOTION = "use_task_motion";

// default
static const std::string DEFAULT_MOVE_ARM_ACTION = "move_arm_action";
static const std::string DEFAULT_PICKUP_ACTION = "pickup_action_service";
static const std::string DEFAULT_PLACE_ACTION = "place_action_service";
static const std::string DEFAULT_GRASP_ACTION = "grasp_action_service";
static const std::string DEFAULT_VISE_ACTION = "vise_action_service";
static const std::string DEFAULT_MATERIAL_LOAD_ACTION= "material_load_action";
static const std::string DEFAULT_MATERIAL_UNLOAD_ACTION= "material_unload_action";
static const std::string DEFAULT_CNC_OPEN_DOOR_ACTION = "cnc_open_door_action";
static const std::string DEFAULT_CNC_CLOSE_DOOR_ACTION = "cnc_close_door_action";
static const std::string DEFAULT_CNC_OPEN_CHUCK_ACTION = "cnc_open_chuck_action";
static const std::string DEFAULT_CNC_CLOSE_CHUCK_ACTION = "cnc_close_chuck_action";
static const std::string DEFAULT_JOINT_TRAJ_ACTION = "joint_trajectory_action";
static const std::string DEFAULT_ROBOT_STATES_TOPIC = "robot_states";
static const std::string DEFAULT_ROBOT_SPINDLE_TOPIC = "robot_spindle";
static const std::string DEFAULT_ROBOT_STATUS_TOPIC = "robot_status";
static const std::string DEFAULT_JOINT_STATE_TOPIC = "joint_states";
static const std::string DEFAULT_EXTERNAL_COMMAND_SERVICE = "external_command";
static const std::string DEFAULT_MATERIAL_HANDLING_STATE_SERVICE = "material_handling_server_state";
static const std::string DEFAULT_TRAJECTORY_FILTER_SERVICE = "filter_trajectory_with_constraints";

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
static const double DURATION_JOINT_MESSAGE_TIMEOUT = 10.0f;
//static const int MAX_WAIT_ATTEMPTS = 40;

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
	return ph.getParam(PARAM_TASK_DESCRIPTION, task_desc_) &&
	                ph.getParam(PARAM_USE_TASK_MOTION, use_task_desc_motion) &&
	                ph.getParam(PARAM_ARM_GROUP,arm_group_) &&
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
	int wait_attempts = 0;

	if(fetch_parameters())
	{
		ROS_INFO_STREAM("Read all parameters successfully");
	}
	else
	{
		ROS_ERROR_STREAM("Failed to read parameters, exiting");
		return false;
	}

	// initializing joint paths (for alternative path execution)
	if (!parseTaskXml(task_desc_, joint_paths_))
	{
	  ROS_ERROR_STREAM("Failed to initialize task xml");
          return false;
	}

	// initializing move arm client
	if(!MoveArmActionClient::setup())
	{
		ROS_ERROR_STREAM("Failed to initialize MoveArmActionClient");
		return false;
	}

	// initializing action service servers
	material_load_server_ptr_ = MaterialLoadServerPtr(new MaterialLoadServer(nh,DEFAULT_MATERIAL_LOAD_ACTION,false));
	material_load_server_ptr_->registerGoalCallback(boost::bind(&StateMachine::material_load_goalcb,this));
	material_unload_server_ptr_ = MaterialUnloadServerPtr(new  MaterialUnloadServer(nh,DEFAULT_MATERIAL_UNLOAD_ACTION,false));
	material_unload_server_ptr_->registerGoalCallback(boost::bind(&StateMachine::material_unload_goalcb,this));

//	material_load_server_ptr_ = MaterialLoadServerPtr(new MaterialLoadServer(nh,DEFAULT_MATERIAL_LOAD_ACTION,
//			boost::bind(&StateMachine::material_load_goalcb,this,_1),false));
//
//	material_unload_server_ptr_ = MaterialUnloadServerPtr(new  MaterialUnloadServer(nh,DEFAULT_MATERIAL_UNLOAD_ACTION,
//					boost::bind(&StateMachine::material_unload_goalcb,this,_1),false));

	// initializing action service clients
	move_pickup_client_ptr_ = MovePickupClientPtr(new MovePickupClient(DEFAULT_PICKUP_ACTION,true));
	move_place_client_ptr_ = MovePlaceClientPtr(new MovePlaceClient(DEFAULT_PLACE_ACTION,true));
	open_door_client_ptr_ =CncOpenDoorClientPtr(new CncOpenDoorClient(DEFAULT_CNC_OPEN_DOOR_ACTION,true));
	close_door_client_ptr_ =CncCloseDoorClientPtr(new CncCloseDoorClient(DEFAULT_CNC_CLOSE_DOOR_ACTION,true));
	open_chuck_client_ptr_ =CncOpenChuckClientPtr(new CncOpenChuckClient(DEFAULT_CNC_OPEN_CHUCK_ACTION,true));
	close_chuck_client_ptr_ =CncCloseChuckClientPtr(new CncCloseChuckClient(DEFAULT_CNC_CLOSE_CHUCK_ACTION,true));
	grasp_action_client_ptr_ = GraspActionClientPtr(new GraspActionClient(DEFAULT_GRASP_ACTION,true));
	vise_action_client_ptr_ = GraspActionClientPtr(new GraspActionClient(DEFAULT_VISE_ACTION,true));
	joint_traj_client_ptr_ = JointTractoryClientPtr(new JointTractoryClient(DEFAULT_JOINT_TRAJ_ACTION,true));

	// initializing publishers
	robot_states_pub_ = nh.advertise<mtconnect_msgs::RobotStates>(DEFAULT_ROBOT_STATES_TOPIC,1);
	robot_spindle_pub_ = nh.advertise<mtconnect_msgs::RobotSpindle>(DEFAULT_ROBOT_SPINDLE_TOPIC,1);

	// initializing subscribers
	robot_status_sub_ = nh.subscribe(DEFAULT_ROBOT_STATUS_TOPIC,1,&StateMachine::ros_status_subs_cb,this);

	// initializing servers
	external_command_srv_ = nh.advertiseService(DEFAULT_EXTERNAL_COMMAND_SERVICE,&StateMachine::external_command_cb,this);

	// initializing clients
	material_server_state_client_ = nh.serviceClient<mtconnect_msgs::MaterialServerState>(DEFAULT_MATERIAL_HANDLING_STATE_SERVICE);
	trajectory_filter_client_ =
	    nh.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(
	        DEFAULT_TRAJECTORY_FILTER_SERVICE);
	// initializing service client req msg
	material_server_state_.request.state_flag = mtconnect_msgs::MaterialServerState::Request::READY;
	material_server_state_.response.accepted = false;


	// initializing mtconnect robot messages
	robot_state_msg_.avail.val = TriState::ENABLED;
	robot_state_msg_.mode.val = RobotMode::AUTO;
	robot_state_msg_.rexec.val = TriState::HIGH;
	robot_spindle_msg_.c_unclamp.val = TriState::HIGH;
	robot_spindle_msg_.s_inter.val = TriState::HIGH;

	// setting up move arm joint goal
	move_arm_joint_goal_.motion_plan_request.group_name = arm_group_;
	move_arm_joint_goal_.motion_plan_request.num_planning_attempts = DEFAULT_PATH_PLANNING_ATTEMPTS;
	move_arm_joint_goal_.planner_service_name = DEFAULT_PATH_PLANNER;
	move_arm_joint_goal_.motion_plan_request.allowed_planning_time = ros::Duration(DURATION_PLANNING_TIME);
	move_arm_joint_goal_.motion_plan_request.planner_id = "";
	move_arm_joint_goal_.motion_plan_request.expected_path_duration = ros::Duration(DURATION_PATH_COMPLETION);

	// initializing timers
	robot_topics_timer_ = nh.createTimer(ros::Duration(DURATION_TIMER_INTERVAL),
			&StateMachine::publish_robot_topics_timercb,this,false,false);

	// initializing material load/unload tasks lists
	material_load_task_sequence_.clear();
	jm_material_load_task_sequence_.clear();
	material_unload_task_sequence_.clear();
	jm_material_unload_task_sequence_.clear();

	using namespace state_machine::tasks;
	using namespace boost::assign;
	material_load_task_sequence_ = list_of((int)MATERIAL_LOAD_START)
			((int)ROBOT_MOVE_HOME)
			((int)CNC_OPEN_DOOR)
			((int)VISE_OPEN)
			((int)CNC_OPEN_CHUCK)
			((int)ROBOT_PICKUP_MATERIAL)
			((int)ROBOT_APPROACH_CNC)
			((int)ROBOT_ENTER_CNC)
			((int)ROBOT_MOVE_TO_CHUCK)
			((int)CNC_CLOSE_CHUCK)
			((int)VISE_CLOSE)
			((int)GRIPPER_OPEN)
			((int)ROBOT_RETREAT_FROM_CHUCK)
			((int)ROBOT_EXIT_CNC)
			((int)CNC_CLOSE_DOOR)
			((int)MATERIAL_LOAD_END);



	jm_material_load_task_sequence_ = list_of((int)MATERIAL_LOAD_START)
	                        ((int)JM_HOME_TO_READY)
	                        ((int)JM_READY_TO_APPROACH)
	                        ((int)GRIPPER_OPEN)
	                        ((int)JM_APPROACH_TO_PICK)
	                        ((int)GRIPPER_CLOSE)
	                        ((int)JM_PICK_TO_DOOR)
	                        ((int)CNC_OPEN_DOOR)
	                        ((int)VISE_OPEN)
	                        ((int)CNC_OPEN_CHUCK)
	                        ((int)JM_DOOR_TO_CHUCK)
	                        ((int)CNC_CLOSE_CHUCK)
	                        ((int)VISE_CLOSE)
                                ((int)GRIPPER_OPEN)
                                ((int)JM_CHUCK_TO_READY)
                                ((int)CNC_CLOSE_DOOR)
	                        ((int)MATERIAL_LOAD_END);


	material_unload_task_sequence_ = list_of((int)MATERIAL_UNLOAD_START)
			((int)ROBOT_APPROACH_CNC)
			((int)GRIPPER_OPEN)
			((int)CNC_OPEN_DOOR)
			((int)ROBOT_ENTER_CNC)
			((int)ROBOT_MOVE_TO_CHUCK)
			((int)GRIPPER_CLOSE)
			((int)VISE_OPEN)
			((int)CNC_OPEN_CHUCK)
			((int)ROBOT_RETREAT_FROM_CHUCK)
			((int)ROBOT_EXIT_CNC)
			((int)ROBOT_PLACE_WORKPIECE)
			((int)CNC_CLOSE_CHUCK)
			((int)VISE_CLOSE)
			((int)CNC_CLOSE_DOOR)
			((int)MATERIAL_UNLOAD_END);

	jm_material_unload_task_sequence_ = list_of((int)MATERIAL_UNLOAD_START)
                            ((int)JM_READY_TO_DOOR)
                            ((int)GRIPPER_OPEN)
                            ((int)CNC_OPEN_DOOR)
                            ((int)JM_DOOR_TO_CHUCK)
                            ((int)GRIPPER_CLOSE)
                            ((int)VISE_OPEN)
                            ((int)CNC_OPEN_CHUCK)
                            ((int)JM_CHUCK_TO_READY)
                            ((int)JM_READY_TO_APPROACH)
                            ((int)JM_APPROACH_TO_PICK)
                            ((int)GRIPPER_OPEN)
                            ((int)JM_PICK_TO_HOME)
                            ((int)MATERIAL_UNLOAD_END);

	// waiting for robot related service servers
	while(	ros::ok() && (
			(!move_arm_client_ptr_->isServerConnected() && !move_arm_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
			(!move_pickup_client_ptr_->isServerConnected() && !move_pickup_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
			(!move_place_client_ptr_->isServerConnected() && !move_place_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
			(!grasp_action_client_ptr_->isServerConnected() && !grasp_action_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
			(!vise_action_client_ptr_->isServerConnected() && !vise_action_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
			(!open_door_client_ptr_->isServerConnected() && !open_door_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
			(!close_door_client_ptr_->isServerConnected() && !close_door_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
			(!open_chuck_client_ptr_->isServerConnected() && !open_chuck_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
			(!close_chuck_client_ptr_->isServerConnected() && !close_chuck_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER))) ||
			(!joint_traj_client_ptr_->isServerConnected() && !joint_traj_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER)))
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

	// starting timers and action servers
	robot_topics_timer_.start();
	material_load_server_ptr_->start();
	material_unload_server_ptr_->start();

	return true;
}

void StateMachine::run()
{
	ros::NodeHandle nh;
	set_active_state(states::STARTUP);

	int last_state = states::EMPTY;
	int active_state = get_active_state();
	print_current_state();
	while(ros::ok() && process_transition())
	{
		ros::spinOnce();

		// getting force fault parameters
		get_param_force_fault_flags();

		// getting externally entered state
		get_param_state_override();

		// printing new state info
		active_state = get_active_state();
		if(active_state != last_state)
		{
			last_state = active_state;
			print_current_state();
		}
	}
}

bool StateMachine::run_next_task()
{
	using namespace mtconnect_cnc_robot_example::state_machine::tasks;

	if(!all_action_servers_connected())
	{
		ROS_WARN_STREAM("One or more action servers are not ready, aborting task");
		current_task_sequence_.clear();
		current_task_index_ = 0;
		return false;
	}

	if(++current_task_index_ < (int)current_task_sequence_.size() )
	{
		std::cout<<"\t - "<< TASK_MAP[current_task_sequence_[current_task_index_]] << " task running\n";
		run_task(current_task_sequence_[current_task_index_]);
		return true;
	}
	else
	{
		// reached end of sequence
		current_task_sequence_.clear();
		current_task_index_ = 0;
		return false;
	}
}

bool StateMachine::run_task(int task_id)
{
	using namespace mtconnect_cnc_robot_example::state_machine::tasks;

	// declaring goal objects
	typedef object_manipulation_msgs::GraspHandPostureExecutionGoal GraspGoal;
	GraspGoal grasp_goal;
	GraspGoal vise_goal;
	mtconnect_msgs::OpenDoorGoal open_door_goal;
	mtconnect_msgs::CloseDoorGoal close_door_goal;
	mtconnect_msgs::OpenChuckGoal open_chuck_goal;
	mtconnect_msgs::CloseChuckGoal close_chuck_goal;
	open_door_goal.open_door = CNC_ACTION_ACTIVE_FLAG;
	close_door_goal.close_door = CNC_ACTION_ACTIVE_FLAG;
	open_chuck_goal.open_chuck = CNC_ACTION_ACTIVE_FLAG;
	close_chuck_goal.close_chuck = CNC_ACTION_ACTIVE_FLAG;

	// clearing cartesian pose array
	cartesian_poses_.poses.clear();
	switch(task_id)
	{
	case MATERIAL_LOAD_END:
		set_active_state(states::MATERIAL_LOAD_COMPLETED);
		break;

	case MATERIAL_UNLOAD_END:
		set_active_state(states::MATERIAL_UNLOAD_COMPLETED);
		break;

	case TEST_TASK_END:

		set_active_state(states::TEST_TASK_COMPLETED);
		break;

	case ROBOT_APPROACH_CNC:

		getTrajectoryInArmSpace(traj_approach_cnc_,cartesian_poses_);
		moveArm(cartesian_poses_);
		set_active_state(states::ROBOT_MOVING);

		break;
	case ROBOT_ENTER_CNC:

		getTrajectoryInArmSpace(traj_enter_cnc_,cartesian_poses_);
		moveArm(cartesian_poses_);
		set_active_state(states::ROBOT_MOVING);

		break;
	case ROBOT_EXIT_CNC:

		getTrajectoryInArmSpace(traj_exit_cnc_,cartesian_poses_);
		moveArm(cartesian_poses_);
		set_active_state(states::ROBOT_MOVING);

		break;
	case ROBOT_MOVE_HOME:

		moveArm(joint_home_pos_);
		set_active_state(states::ROBOT_MOVING);
		break;

	case ROBOT_MOVE_TO_CHUCK:

		getTrajectoryInArmSpace(traj_move_to_chuck_,cartesian_poses_);
		moveArm(cartesian_poses_);
		set_active_state(states::ROBOT_MOVING);
		break;

	case ROBOT_RETREAT_FROM_CHUCK:

		getTrajectoryInArmSpace(traj_retreat_from_chuck_,cartesian_poses_);
		moveArm(cartesian_poses_);
		set_active_state(states::ROBOT_MOVING);
		break;

	case ROBOT_PICKUP_MATERIAL:

		move_pickup_client_ptr_->sendGoal(material_load_pickup_goal_);
		set_active_state(states::ROBOT_MOVING);
		break;

	case ROBOT_PLACE_WORKPIECE:

		move_place_client_ptr_->sendGoal(material_unload_place_goal_);
		set_active_state(states::ROBOT_MOVING);
		break;

	case ROBOT_CARTESIAN_MOVE:

		getTrajectoryInArmSpace(traj_arbitrary_move_,cartesian_poses_);
		moveArm(cartesian_poses_);
		set_active_state(states::ROBOT_MOVING);
		break;

	case CNC_CLOSE_CHUCK:

		close_chuck_client_ptr_->sendGoal(close_chuck_goal);
		set_active_state(states::CNC_MOVING);
		break;

	case CNC_OPEN_CHUCK:

		open_chuck_client_ptr_->sendGoal(open_chuck_goal);
		set_active_state(states::CNC_MOVING);
		break;

	case CNC_CLOSE_DOOR:

		close_door_client_ptr_->sendGoal(close_door_goal);
		set_active_state(states::CNC_MOVING);
		break;

	case CNC_OPEN_DOOR:

		open_door_client_ptr_->sendGoal(open_door_goal);
		set_active_state(states::CNC_MOVING);
		break;

	case GRIPPER_CLOSE:

		grasp_goal.goal = GraspGoal::GRASP;
		grasp_action_client_ptr_->sendGoal(grasp_goal);
		set_active_state(states::GRIPPER_MOVING);
		break;

	case GRIPPER_OPEN:

		grasp_goal.goal = GraspGoal::RELEASE;
		grasp_action_client_ptr_->sendGoal(grasp_goal);
		set_active_state(states::GRIPPER_MOVING);
		break;
	case VISE_CLOSE:

		vise_goal.goal = GraspGoal::GRASP;
		vise_action_client_ptr_->sendGoal(vise_goal);
		set_active_state(states::GRIPPER_MOVING);
		break;

	case VISE_OPEN:

		vise_goal.goal = GraspGoal::RELEASE;
		vise_action_client_ptr_->sendGoal(vise_goal);
		set_active_state(states::GRIPPER_MOVING);
		break;


        case JM_HOME_TO_READY:
        case JM_READY_TO_APPROACH:
        case JM_APPROACH_TO_PICK:
        case JM_PICK_TO_DOOR:
        case JM_DOOR_TO_CHUCK:
        case JM_READY_TO_DOOR:
        case JM_CHUCK_TO_READY:
        case JM_PICK_TO_HOME:
	        // The task ID for all JM moves is also the key for
	        // the task name.  The task name is passed to move
	        // arm and it executes the path from the task_description
	        moveArm(TASK_MAP[task_id]);
                set_active_state(states::ROBOT_MOVING);
                break;

	default:

		break;
	}

	return true;
}

void StateMachine::cancel_active_material_requests()
{
	if(material_load_server_ptr_->isActive())
	{
		MaterialLoadServer::Result res;
		res.load_state= "Failed";
		material_load_server_ptr_->setAborted(res,res.load_state);
		ROS_INFO_STREAM("Material Load goal aborted");
	}

	if(material_unload_server_ptr_->isActive())
	{
		MaterialUnloadServer::Result res;
		res.unload_state = "Failed";
		material_unload_server_ptr_->setAborted(res,res.unload_state);
		ROS_INFO_STREAM("Material Unload goal aborted");
	}
}

void StateMachine::cancel_active_action_goals()
{
	move_pickup_client_ptr_->cancelAllGoals();
	move_arm_client_ptr_->cancelAllGoals();
	move_place_client_ptr_->cancelAllGoals();
	joint_traj_client_ptr_->cancelAllGoals();

	open_door_client_ptr_->cancelAllGoals();
	open_chuck_client_ptr_->cancelAllGoals();
	close_door_client_ptr_->cancelAllGoals();
	close_chuck_client_ptr_->cancelAllGoals();

	grasp_action_client_ptr_->cancelAllGoals();
	vise_action_client_ptr_->cancelAllGoals();
}

bool StateMachine::all_action_servers_connected()
{
	// action clients
	if(!(move_pickup_client_ptr_->isServerConnected() && move_place_client_ptr_->isServerConnected()&&
			move_arm_client_ptr_->isServerConnected() && joint_traj_client_ptr_->isServerConnected()))
	{
		set_active_state(states::ROBOT_FAULT);
		return false;
	}

	if(!(open_door_client_ptr_->isServerConnected() &&	close_door_client_ptr_->isServerConnected() &&
			open_chuck_client_ptr_->isServerConnected() && close_chuck_client_ptr_->isServerConnected()))
	{
		set_active_state(states::CNC_FAULT);
		return false;
	}

	if(!grasp_action_client_ptr_->isServerConnected() && !vise_action_client_ptr_->isServerConnected())
	{
		set_active_state(states::GRIPPER_FAULT);
		return false;
	}

	return true;

}

// transition actions
bool StateMachine::on_startup()
{
	return setup();

}

bool StateMachine::on_ready()
{
	// communicating ready state with service call
	if(!material_server_state_.response.accepted)
	{
		// sending ready state to server
		if(material_server_state_client_.exists() &&
				material_server_state_client_.call(material_server_state_.request,material_server_state_.response))
		{
			ROS_INFO_STREAM("server state service call "<< (material_server_state_.response.accepted ? "accepted" : "rejected"));
		}
		else
		{
			ROS_WARN_STREAM("server state service call failed");
			ros::Duration(DURATION_LOOP_PAUSE).sleep();
		}
	}

	return material_server_state_.response.accepted;
}

bool StateMachine::on_robot_reset()
{
	// resetting service request accepted flag back to false
	material_server_state_.response.accepted = false;
	return true;
}

bool StateMachine::on_material_load_started()
{
	current_task_index_ = 0;
	if( use_task_desc_motion)
        {
          ROS_INFO("Executing NEW dumb joint move material load");
          current_task_sequence_.assign(jm_material_load_task_sequence_.begin(), jm_material_load_task_sequence_.end());
        }

	else
	{
          ROS_INFO("Executing OLD smart path planning material load");
          current_task_sequence_.assign(material_load_task_sequence_.begin(),material_load_task_sequence_.end());
        }
	run_next_task();
	return true;
}

bool StateMachine::on_test_task_started()
{
	// cancelling all ongoing tasks
	cancel_active_action_goals();
	cancel_active_material_requests();

	current_task_index_ = 0;
	current_task_sequence_.clear();
	current_task_sequence_.push_back(tasks::TEST_TASK_START);
	current_task_sequence_.push_back(test_task_id_);
	current_task_sequence_.push_back(tasks::TEST_TASK_END);
	run_next_task();

	return true;
}

bool StateMachine::on_test_task_completed()
{
	return true;
}

bool StateMachine::on_material_load_completed()
{
	MaterialLoadServer::Result res;
	res.load_state= "Succeeded";

	if(material_load_server_ptr_->isActive())
	{
		material_load_server_ptr_->setSucceeded(res);
		ROS_INFO_STREAM("MATERIAL LOAD request succeeded");
	}


	return true;
}

bool StateMachine::on_material_unload_started()
{
	current_task_index_ = 0;

	if( use_task_desc_motion)
	        {
	          ROS_INFO("Executing NEW dumb joint move material unload");
	          current_task_sequence_.assign(jm_material_unload_task_sequence_.begin(), jm_material_unload_task_sequence_.end());
	        }

	        else
	        {
	          ROS_INFO("Executing OLD smart path planning material unload");
	          current_task_sequence_.assign(material_unload_task_sequence_.begin(),material_unload_task_sequence_.end());
	        }


	run_next_task();
	return true;
}

bool StateMachine::on_material_unload_completed()
{
	MaterialUnloadServer::Result res;
	res.unload_state= "Succeeded";
	if(material_unload_server_ptr_->isActive())
	{
		material_unload_server_ptr_->setSucceeded(res);
		ROS_INFO_STREAM("MATERIAL UNLOAD request succeeded");
	}

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

bool StateMachine::on_robot_moving()
{
	using namespace mtconnect_cnc_robot_example::state_machine::tasks;

	// check if task is set to trigger fault
	if(get_param_fault_on_task_check(current_task_sequence_[current_task_index_]))
	{
		ROS_WARN_STREAM("Forcing fault on task "<<tasks::TASK_MAP[current_task_sequence_[current_task_index_]]);
		set_active_state(states::ROBOT_FAULT);
		return true;
	}

	int state = actionlib::SimpleClientGoalState::ACTIVE;
	switch(current_task_sequence_[current_task_index_])
	{
	case ROBOT_PICKUP_MATERIAL:

		state = move_pickup_client_ptr_->getState().state_;
		break;

	case ROBOT_PLACE_WORKPIECE:

		state = move_place_client_ptr_->getState().state_;
		break;

	default: // all arm client tasks

	        if( use_task_desc_motion )
	        {
	          state = joint_traj_client_ptr_->getState().state_;
	        }
	        else
	        {
	          state = move_arm_client_ptr_->getState().state_;
	        }
		break;
	}

	// examining state
	switch(state)
	{
	case actionlib::SimpleClientGoalState::ACTIVE:
		break;

	case actionlib::SimpleClientGoalState::SUCCEEDED:
		run_next_task();
		break;

	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::ABORTED:

		set_active_state(states::ROBOT_FAULT);
		break;
	}

	return true;
}

bool StateMachine::on_cnc_moving()
{
	using namespace mtconnect_cnc_robot_example::state_machine::tasks;

	// check if task is set to trigger fault
	if(get_param_fault_on_task_check(current_task_sequence_[current_task_index_]))
	{
		ROS_WARN_STREAM("Forcing fault on task "<<tasks::TASK_MAP[current_task_sequence_[current_task_index_]]);
		set_active_state(states::CNC_FAULT);
		return true;
	}

	int state = actionlib::SimpleClientGoalState::ACTIVE;
	switch(current_task_sequence_[current_task_index_])
	{
		case CNC_CLOSE_CHUCK:

			state = close_chuck_client_ptr_->getState().state_;
			break;

		case CNC_OPEN_CHUCK:

			state = open_chuck_client_ptr_->getState().state_;
			break;

		case CNC_CLOSE_DOOR:

			state = close_door_client_ptr_->getState().state_;
			break;

		case CNC_OPEN_DOOR:

			state = open_door_client_ptr_->getState().state_;
			break;
	}

	// examining state
	switch(state)
	{
	case actionlib::SimpleClientGoalState::ACTIVE:
		break;

	case actionlib::SimpleClientGoalState::SUCCEEDED:
		run_next_task();
		break;

	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::ABORTED:

		set_active_state(states::CNC_FAULT);
		break;
	}

	return true;
}

bool StateMachine::on_gripper_moving()
{
	using namespace mtconnect_cnc_robot_example::state_machine::tasks;

	// check if task is set to trigger fault
	if(get_param_fault_on_task_check(current_task_sequence_[current_task_index_]))
	{
		ROS_WARN_STREAM("Forcing fault on task "<<tasks::TASK_MAP[current_task_sequence_[current_task_index_]]);
		set_active_state(states::GRIPPER_FAULT);
		return true;
	}

	int state = actionlib::SimpleClientGoalState::ACTIVE;
	switch(current_task_sequence_[current_task_index_])
	{
		case GRIPPER_CLOSE:
		case GRIPPER_OPEN:
			state = grasp_action_client_ptr_->getState().state_;
			break;

		case VISE_CLOSE:
		case VISE_OPEN:
			state = vise_action_client_ptr_->getState().state_;
			break;
	}

	// examining state
	switch(state)
	{
	case actionlib::SimpleClientGoalState::ACTIVE:
		break;

	case actionlib::SimpleClientGoalState::SUCCEEDED:
		run_next_task();
		break;

	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::ABORTED:

		set_active_state(states::GRIPPER_FAULT);
		break;
	}

	return true;
}

bool StateMachine::on_robot_fault()
{
	cancel_active_material_requests();
	cancel_active_action_goals();
	return true;
}

bool StateMachine::on_cnc_fault()
{
	cancel_active_material_requests();
	cancel_active_action_goals();
	return true;
}

bool StateMachine::on_gripper_fault()
{
	cancel_active_material_requests();
	cancel_active_action_goals();
	return true;
}

// fault handling related methods
void StateMachine::get_param_force_fault_flags()
{
	ros::NodeHandle nh("~");

	bool force_robot_fault, force_cnc_fault, force_gripper_fault;
	if(nh.getParam(PARAM_FORCE_ROBOT_FAULT,force_robot_fault) && force_robot_fault)
	{
		ROS_INFO_STREAM("Forcing 'ROBOT_FAULT'");
		set_active_state(states::ROBOT_FAULT);
		nh.setParam(PARAM_FORCE_ROBOT_FAULT,false);
	}

	if(nh.getParam(PARAM_FORCE_CNC_FAULT,force_cnc_fault) && force_cnc_fault)
	{
		ROS_INFO_STREAM("Forcing 'CNC_FAULT'");
		set_active_state(states::CNC_FAULT);
		nh.setParam(PARAM_FORCE_CNC_FAULT,false);
	}

	if(nh.getParam(PARAM_FORCE_GRIPPER_FAULT,force_gripper_fault) && force_gripper_fault)
	{
		ROS_INFO_STREAM("Forcing 'GRIPPER_FAULT'");
		set_active_state(states::GRIPPER_FAULT);
		nh.setParam(PARAM_FORCE_GRIPPER_FAULT,false);
	}

}

bool StateMachine::get_param_fault_on_task_check(int task_id)
{
	ros::NodeHandle nh("~");
	int fault_on_task_id;

	// check task id match
	if(nh.getParam(PARAM_FORCE_FAULT_ON_TASK,fault_on_task_id) && fault_on_task_id != tasks::NO_TASK &&
			task_id == fault_on_task_id )
	{
		//nh.setParam(PARAM_FORCE_FAULT_ON_TASK,tasks::NO_TASK);
		return true;
	}
	else
	{
		return false;
	}
}

bool StateMachine::check_arm_at_position(sensor_msgs::JointState &joints, double tolerance)
{
//	boost::shared_ptr<sensor_msgs::JointState> actual_joints_ptr;
	sensor_msgs::JointStateConstPtr actual_joints_ptr;
	sensor_msgs::JointState actual_joints;
	bool success = true;
	int joint_index = 0;

	// listening for joint state topic
	ros::NodeHandle nh;
	actual_joints_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(DEFAULT_JOINT_STATE_TOPIC,nh
			,ros::Duration(DURATION_JOINT_MESSAGE_TIMEOUT));

	if(actual_joints_ptr.get() != NULL /*null pointer*/)
	{
		// finding matching joint names
		for(int i = 0; i < joints.name.size(); i++)
		{
			std::string &name = joints.name[i];
			std::vector<std::string>::const_iterator iter_pos =  std::find(actual_joints_ptr->name.begin(),
					actual_joints_ptr->name.end(),name);

			if(iter_pos != actual_joints_ptr->name.end())
			{
				joint_index = 0;
				while(actual_joints_ptr->name[joint_index].compare(name)!=0)
				{
					joint_index++;
				}

				// comparing joint values
				std::vector<double> &vals = joints.position;
				const std::vector<double> &true_vals = actual_joints_ptr->position;
				if( (joint_index >= joints.name.size()) ||  (std::abs(vals[i] - true_vals[joint_index]) > tolerance))
				{
					success = false;
					break;
				}
			}
			else
			{
				ROS_ERROR_STREAM("Joint "<<name<<" not found, failed joint position check");
				success = false;
				break;
			}
		}

	}
	else
	{
		ROS_ERROR_STREAM("Joint state message not received, failed joint position check");
		success = false;
	}

	return success;
}

// callbacks
void StateMachine::material_load_goalcb(/*const MaterialLoadServer::GoalConstPtr &gh*/)
{

	if(get_active_state()== states::READY)
	{
		material_load_server_ptr_->acceptNewGoal();
		set_active_state(states::MATERIAL_LOAD_STARTED);
	}

}

void StateMachine::material_unload_goalcb(/*const MaterialUnloadServer::GoalConstPtr &gh*/)
{
	if(get_active_state()== states::READY)
	{
		material_unload_server_ptr_->acceptNewGoal();
		set_active_state(states::MATERIAL_UNLOAD_STARTED);
	}
}

void StateMachine::ros_status_subs_cb(const industrial_msgs::RobotStatusConstPtr &msg)
{
	if(msg->e_stopped.val == industrial_msgs::TriState::ENABLED)
	{
		ROS_INFO_STREAM("Received 'e_stopped' enabled, going to ROBOT_FAULT");
		set_active_state(states::ROBOT_FAULT);
	}
}

bool StateMachine::external_command_cb(mtconnect_cnc_robot_example::Command::Request &req,
				mtconnect_cnc_robot_example::Command::Response &res)
{
	//using namespace mtconnect_cnc_robot_example;
	switch(req.command_flag)
	{
	case Command::Request::FAULT_RESET:

		if(get_active_state() == states::ROBOT_FAULT && check_arm_at_position(joint_home_pos_,DEFAULT_JOINT_ERROR_TOLERANCE))
		{
			set_active_state(states::ROBOT_RESET);
			res.accepted = true;
			ROS_INFO_STREAM("Fault reset accepted");
		}
		else
		{
			res.accepted = false;
			ROS_INFO_STREAM("Fault reset rejected");
		}

		break;
	case Command::Request::RUN_TASK:
		test_task_id_ = req.task_id;
		set_active_state(states::TEST_TASK_STARTED);
		res.accepted = true;
		ROS_INFO_STREAM("Run task accepted");
		break;

	case Command::Request::START:
			break;
	case Command::Request::EXIT:
			break;
	default:
		break;
	}

	return true;
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
	move_arm_joint_goal_.motion_plan_request.goal_constraints.joint_constraints.clear();
	joint_info.toJointConstraints(DEFAULT_JOINT_ERROR_TOLERANCE,DEFAULT_JOINT_ERROR_TOLERANCE,
			move_arm_joint_goal_.motion_plan_request.goal_constraints.joint_constraints);

	// sending goal
	move_arm_client_ptr_->sendGoal(move_arm_joint_goal_);
	return true;
}

// joint trajectory move arm method
bool StateMachine::moveArm(std::string & move_name)
{
  joint_traj_goal_.trajectory = *joint_paths_[move_name];
  ROS_INFO_STREAM("Sending a joint trajectory with " <<
                  joint_traj_goal_.trajectory.points.size() << "points");
  trajectory_filter_.request.trajectory = joint_traj_goal_.trajectory;
  if(!joint_traj_goal_.trajectory.points.empty())
    if(trajectory_filter_client_.call(trajectory_filter_))
    {
        if (trajectory_filter_.response.error_code.val ==
            trajectory_filter_.response.error_code.SUCCESS)
        {
          ROS_INFO("Trajectory successfully filtered...sending goal");
          joint_traj_goal_.trajectory = trajectory_filter_.response.trajectory;
          ROS_INFO("Copying trajectory to back to goal");
          joint_traj_client_ptr_->sendGoal(joint_traj_goal_);
        }
        else
        {
          ROS_ERROR("Failed to process filter trajectory, entering fault state");
          set_active_state(states::ROBOT_FAULT);
        }
      }

    else
    {
      ROS_ERROR("Failed to call filter trajectory, entering fault state");
      set_active_state(states::ROBOT_FAULT);
    }
  else
  {
    ROS_ERROR_STREAM(move_name << " trajectory is empty, failing");
    set_active_state(states::ROBOT_FAULT);
  }

  return true;
}

