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

#include <mtconnect_state_machine/state_machine.h>
#include <mtconnect_state_machine/utilities.h>
#include <industrial_robot_client/utils.h>

using namespace mtconnect_state_machine;

static const std::string PARAM_TASK_DESCRIPTION = "task_description";
static const std::string PARAM_FORCE_FAULT_STATE = "force_fault";
static const std::string PARAM_STATE_OVERRIDE = "state_override";
static const std::string PARAM_LOOP_RATE = "loop_rate";
static const std::string PARAM_CHECK_ENABLED = "home_check";
static const std::string PARAM_HOME_TOL = "home_tol";
static const std::string KEY_HOME_POSITION = "home";

// Material load moves
static const std::string KEY_JM_HOME_TO_APPROACH = "JM_HOME_TO_APPROACH";
static const std::string KEY_JM_APPROACH_TO_PICK = "JM_APPROACH_TO_PICK";
static const std::string KEY_JM_PICK_TO_CHUCK = "JM_PICK_TO_CHUCK";
static const std::string KEY_JM_CHUCK_TO_DOOR = "JM_CHUCK_TO_DOOR";
static const std::string KEY_JM_DOOR_TO_HOME = "JM_DOOR_TO_HOME";

// Material unload moves
static const std::string KEY_JM_HOME_TO_DOOR = "JM_HOME_TO_DOOR";
static const std::string KEY_JM_DOOR_TO_CHUCK = "JM_DOOR_TO_CHUCK";
static const std::string KEY_JM_CHUCK_TO_DROP = "JM_CHUCK_TO_DROP";
static const std::string KEY_JM_DROP_TO_HOME = "JM_DROP_TO_HOME";

static const std::string DEFAULT_GRASP_ACTION = "gripper_action_service";
static const std::string DEFAULT_VISE_ACTION = "vise_action_service";
static const std::string DEFAULT_MATERIAL_LOAD_ACTION = "material_load_action";
static const std::string DEFAULT_MATERIAL_UNLOAD_ACTION = "material_unload_action";
static const std::string DEFAULT_CNC_OPEN_DOOR_ACTION = "cnc_open_door_action";
static const std::string DEFAULT_CNC_CLOSE_DOOR_ACTION = "cnc_close_door_action";
static const std::string DEFAULT_CNC_OPEN_CHUCK_ACTION = "cnc_open_chuck_action";
static const std::string DEFAULT_CNC_CLOSE_CHUCK_ACTION = "cnc_close_chuck_action";
static const std::string DEFAULT_JOINT_TRAJ_ACTION = "joint_trajectory_action";
static const std::string DEFAULT_ROBOT_STATES_TOPIC = "robot_states";
static const std::string DEFAULT_ROBOT_SPINDLE_TOPIC = "robot_spindle";
static const std::string DEFAULT_ROBOT_STATUS_TOPIC = "robot_status";
static const std::string DEFAULT_JOINT_STATE_TOPIC = "joint_states";
static const std::string DEFAULT_SM_STATUS_TOPIC = "state_machine_status";
static const std::string DEFAULT_EXTERNAL_COMMAND_SERVICE = "external_command";
static const std::string DEFAULT_MATERIAL_LOAD_SET_STATE_SERVICE = "/MaterialLoad/set_mtconnect_state";
static const std::string DEFAULT_MATERIAL_UNLOAD_SET_STATE_SERVICE = "/MaterialUnload/set_mtconnect_state";
static const std::string DEFAULT_TRAJECTORY_FILTER_SERVICE = "filter_trajectory_with_constraints";

static const std::string MTCONNECT_ACTION_ACTIVE_FLAG = "ACTIVE";

//convienence typdef for getting to mtconnect state (i.e. ready, not, ready, etc...)
typedef mtconnect_msgs::SetMTConnectState::Request MtConnectState;

StateMachine::StateMachine() :
    nh_()
{
  state_ = StateTypes::INVALID;
}

StateMachine::~StateMachine()
{

}

bool StateMachine::init()
{
  ros::NodeHandle ph("~");
  std::string task_desc;

  if (!ph.getParam(PARAM_LOOP_RATE, loop_rate_))
  {
    ROS_WARN_STREAM("Param: " << PARAM_LOOP_RATE << " not set, using default");
    loop_rate_ = 10;
  }
  if (!ph.getParam(PARAM_CHECK_ENABLED, home_check_))
  {
    ROS_WARN_STREAM("Param: " << PARAM_CHECK_ENABLED << "not set, setting enabled");
    home_check_ = true;
  }
  if (!ph.getParam(PARAM_HOME_TOL, home_tol_))
  {
    ROS_WARN_STREAM("Param: " << PARAM_HOME_TOL << " not set, using default");
    home_tol_ = 0.1; //radians
  }
  if (!ph.getParam(PARAM_TASK_DESCRIPTION, task_desc))
  {
    ROS_ERROR("Failed to load task description parameter");
    return false;
  }

  std::map<std::string, boost::shared_ptr<mtconnect::JointPoint> > points;
  if (!parseTaskXml(task_desc, joint_paths_, points))
  {
    ROS_ERROR("Failed to parse xml");
    return false;
  }

  if (points.empty())
  {
    ROS_ERROR("Failed to find defined points");
    return false;
  }

  ROS_INFO_STREAM("Adding home position from task description");
  home_ = points[KEY_HOME_POSITION];
  if (home_->values_.empty())
  {
    ROS_ERROR("Home position is empty, failed to load home");
    return false;
  }

  // initializing action service servers
  material_load_server_ptr_ = MaterialLoadServerPtr(new MaterialLoadServer(nh_, DEFAULT_MATERIAL_LOAD_ACTION, false));
  material_load_server_ptr_->registerGoalCallback(boost::bind(&StateMachine::materialLoadGoalCB, this));
  material_unload_server_ptr_ = MaterialUnloadServerPtr(
      new MaterialUnloadServer(nh_, DEFAULT_MATERIAL_UNLOAD_ACTION, false));
  material_unload_server_ptr_->registerGoalCallback(boost::bind(&StateMachine::materialUnloadGoalCB, this));

  // initializing action service clients
  open_door_client_ptr_ = CncOpenDoorClientPtr(new CncOpenDoorClient(DEFAULT_CNC_OPEN_DOOR_ACTION, false));
  close_door_client_ptr_ = CncCloseDoorClientPtr(new CncCloseDoorClient(DEFAULT_CNC_CLOSE_DOOR_ACTION, false));
  open_chuck_client_ptr_ = CncOpenChuckClientPtr(new CncOpenChuckClient(DEFAULT_CNC_OPEN_CHUCK_ACTION, false));
  close_chuck_client_ptr_ = CncCloseChuckClientPtr(new CncCloseChuckClient(DEFAULT_CNC_CLOSE_CHUCK_ACTION, false));
  grasp_action_client_ptr_ = GraspActionClientPtr(new GraspActionClient(DEFAULT_GRASP_ACTION, false));
  vise_action_client_ptr_ = GraspActionClientPtr(new GraspActionClient(DEFAULT_VISE_ACTION, false));
  joint_traj_client_ptr_ = JointTractoryClientPtr(new JointTractoryClient(DEFAULT_JOINT_TRAJ_ACTION, false));

  // initializing publishers
  robot_states_pub_ = nh_.advertise<mtconnect_msgs::RobotStates>(DEFAULT_ROBOT_STATES_TOPIC, 1);
  robot_spindle_pub_ = nh_.advertise<mtconnect_msgs::RobotSpindle>(DEFAULT_ROBOT_SPINDLE_TOPIC, 1);
  state_machine_pub_ = nh_.advertise<mtconnect_example_msgs::StateMachineStatus>(DEFAULT_SM_STATUS_TOPIC, 1);

  // initializing subscribers
  robot_status_sub_ = nh_.subscribe(DEFAULT_ROBOT_STATUS_TOPIC, 1, &StateMachine::robotStatusCB, this);
  joint_states_sub_ = nh_.subscribe(DEFAULT_JOINT_STATE_TOPIC, 1, &StateMachine::jointStatesCB, this);

  // initializing servers
  external_command_srv_ = nh_.advertiseService(DEFAULT_EXTERNAL_COMMAND_SERVICE, &StateMachine::externalCommandCB,
                                               this);

  // initializing clients
  material_load_set_state_client_ = nh_.serviceClient<mtconnect_msgs::SetMTConnectState>(
      DEFAULT_MATERIAL_LOAD_SET_STATE_SERVICE);
  material_unload_set_state_client_ = nh_.serviceClient<mtconnect_msgs::SetMTConnectState>(
      DEFAULT_MATERIAL_UNLOAD_SET_STATE_SERVICE);

  trajectory_filter_client_ = nh_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(
      DEFAULT_TRAJECTORY_FILTER_SERVICE);

  // starting action servers
  material_load_server_ptr_->start();
  material_unload_server_ptr_->start();

  setState(StateTypes::IDLE);

  return true;

}

void StateMachine::run()
{
  ROS_INFO_STREAM("Entering blocking run");
  ros::Rate r(loop_rate_);
  while (ros::ok())
  {
    //ROS_INFO_STREAM_THROTTLE(5, "Begin blocking run loop, state: " << state_);
    runOnce();
    callPublishers();
    ros::spinOnce();
    errorChecks();
    overrideChecks();
    //ROS_INFO_STREAM_THROTTLE(5, "End blocking run loop, state: " << state_);
    r.sleep();
  }

}

void StateMachine::runOnce()
{
  //TODO: Remove these variables (they can't be used under a case statement)
  MaterialLoadServer::Result load_res;
  MaterialUnloadServer::Result unload_res;

  switch (state_)
  {
    case StateTypes::IDLE:
      break;

    case StateTypes::INITING:
      setState(StateTypes::CHECK_HOME);
      break;

    case StateTypes::CHECK_HOME:
      if ( isHome())
      {
        ROS_INFO_STREAM("Robot start home check passed");
        setState(StateTypes::WAIT_FOR_ACTIONS);
      }
      else
      {
        ROS_ERROR_STREAM("Robot not in home state on start");
        setState(StateTypes::ABORTING);
      }
      break;

    case StateTypes::WAIT_FOR_ACTIONS:
      if (areActionsReady())
      {
        setState(StateTypes::WAIT_FOR_SERVICES);
      }
      else
      {
        ROS_INFO_STREAM_THROTTLE(20, "Waiting for actions to be ready");
      }
      break;

    case StateTypes::WAIT_FOR_SERVICES:
      if (areServicesReady())
      {
        setState(StateTypes::SET_MAT_ACTIONS_READY);
      }
      else
      {
        ROS_INFO_STREAM_THROTTLE(20, "Waiting for services to be ready");
      }
      break;

    case StateTypes::SET_MAT_ACTIONS_READY:
      if (setMatActionsReady())
      {
        setState(StateTypes::WAITING);
      }
      else
      {
        ROS_ERROR("Failed to set actions ready, aborting");
        setState(StateTypes::ABORTING);
      }
      break;

    case StateTypes::WAITING:
      ROS_INFO_STREAM_THROTTLE(20, "Waiting for request");
      break;

    case StateTypes::MATERIAL_LOADING:
      ROS_INFO_STREAM("++++++++++++++++++++++++ LOADING MATERIAL ++++++++++++++++++++++++");
      setState(StateTypes::ML_MOVE_PICK_APPROACH);
      break;

    case StateTypes::ML_MOVE_PICK_APPROACH:
      moveArm(KEY_JM_HOME_TO_APPROACH);
      openGripper();
      openDoor();
      setState(StateTypes::ML_WAIT_MOVE_PICK_APPROACH);
      break;

    case StateTypes::ML_WAIT_MOVE_PICK_APPROACH:
      if (isMoveDone() && isGripperOpened())
      {
        setState(StateTypes::ML_MOVE_PICK);
      }
      break;

    case StateTypes::ML_MOVE_PICK:
      moveArm(KEY_JM_APPROACH_TO_PICK);
      setState(StateTypes::ML_WAIT_MOVE_PICK);
      break;

    case StateTypes::ML_WAIT_MOVE_PICK:
      if (isMoveDone())
      {
        setState(StateTypes::ML_PICK);
      }
      break;

    case StateTypes::ML_PICK:
      closeGripper();
      setState(StateTypes::ML_WAIT_PICK);
      break;

    case StateTypes::ML_WAIT_PICK:
      if (isGripperClosed() && isDoorOpened())
      {
        setState(StateTypes::ML_MOVE_CHUCK);
      }
      break;

    case StateTypes::ML_MOVE_CHUCK:
      moveArm(KEY_JM_PICK_TO_CHUCK);
      setState(StateTypes::ML_WAIT_MOVE_CHUCK);
      break;

    case StateTypes::ML_WAIT_MOVE_CHUCK:
      if (isMoveDone())
      {
        setState(StateTypes::ML_CLOSE_CHUCK);
      }
      break;

    case StateTypes::ML_CLOSE_CHUCK:
      closeChuck();
      setState(StateTypes::ML_WAIT_CLOSE_CHUCK);
      break;

    case StateTypes::ML_WAIT_CLOSE_CHUCK:
      if (isChuckClosed())
      {
        setState(StateTypes::ML_RELEASE_PART);
      }
      break;

    case StateTypes::ML_RELEASE_PART:
      openGripper();
      setState(StateTypes::ML_WAIT_RELEASE_PART);
      break;

    case StateTypes::ML_WAIT_RELEASE_PART:
      if (isGripperOpened())
      {
        setState(StateTypes::ML_MOVE_DOOR);
      }
      break;

    case StateTypes::ML_MOVE_DOOR:
      moveArm(KEY_JM_CHUCK_TO_DOOR);
      setState(StateTypes::ML_WAIT_MOVE_DOOR);
      break;

    case StateTypes::ML_WAIT_MOVE_DOOR:
      if (isMoveDone())
      {
        setState(StateTypes::ML_MOVE_HOME);
      }
      break;

    case StateTypes::ML_MOVE_HOME:
      moveArm(KEY_JM_DOOR_TO_HOME);
      closeDoor();
      setState(StateTypes::ML_WAIT_MOVE_HOME);
      break;

    case StateTypes::ML_WAIT_MOVE_HOME:
      if (isMoveDone() && isDoorClosed())
      {
        setState(StateTypes::MATERIAL_LOADED);
      }
      break;

    case StateTypes::MATERIAL_LOADED:
      ROS_INFO_STREAM("Material loaded");
      load_res.load_state = "Succeeded";
      material_load_server_ptr_->setSucceeded(load_res);
      setState(StateTypes::WAITING);
      break;




    case StateTypes::MATERIAL_UNLOADING:
      ROS_INFO_STREAM("++++++++++++++++++++++++ UNLOADING MATERIAL ++++++++++++++++++++++++");
      setState(StateTypes::MU_MOVE_DOOR);
      break;

    case StateTypes::MU_MOVE_DOOR:
      moveArm(KEY_JM_HOME_TO_DOOR);
      openDoor();
      setState(StateTypes::MU_WAIT_MOVE_DOOR);
      break;

    case StateTypes::MU_WAIT_MOVE_DOOR:
      if(isMoveDone() && isDoorOpened())
      {
        setState(StateTypes::MU_MOVE_CHUCK);
      }
      break;

    case StateTypes::MU_MOVE_CHUCK:
      moveArm(KEY_JM_DOOR_TO_CHUCK);
      setState(StateTypes::MU_WAIT_MOVE_CHUCK);
      break;

    case StateTypes::MU_WAIT_MOVE_CHUCK:
      if(isMoveDone())
      {
        setState(StateTypes::MU_PICK_PART);
      }
      break;

    case StateTypes::MU_PICK_PART:
      closeGripper();
      setState(StateTypes::MU_WAIT_PICK_PART);
      break;

    case StateTypes::MU_WAIT_PICK_PART:
      if(isGripperClosed())
      {
        setState(StateTypes::MU_OPEN_CHUCK);
      }
      break;

    case StateTypes::MU_OPEN_CHUCK:
      openChuck();
      setState(StateTypes::MU_WAIT_OPEN_CHUCK);
      break;

    case StateTypes::MU_WAIT_OPEN_CHUCK:
      if(isChuckOpened())
      {
        setState(StateTypes::MU_MOVE_DROP);
      }
      break;

    case StateTypes::MU_MOVE_DROP:
      moveArm(KEY_JM_CHUCK_TO_DROP);
      setState(StateTypes::MU_WAIT_MOVE_DROP);
      break;

    case StateTypes::MU_WAIT_MOVE_DROP:
      if(isMoveDone())
      {
        setState(StateTypes::MU_DROP);
      }
      break;

    case StateTypes::MU_DROP:
      openGripper();
      setState(StateTypes::MU_WAIT_DROP);
      break;

    case StateTypes::MU_WAIT_DROP:
      if(isGripperOpened())
      {
        setState(StateTypes::MU_MOVE_HOME);
      }
      break;

    case StateTypes::MU_MOVE_HOME:
      moveArm(KEY_JM_DROP_TO_HOME);
      setState(StateTypes::MU_WAIT_MOVE_HOME);
      break;

    case StateTypes::MU_WAIT_MOVE_HOME:
      if(isMoveDone())
      {
        setState(StateTypes::MATERIAL_UNLOADED);
      }
      break;

    case StateTypes::MATERIAL_UNLOADED:
      ROS_INFO_STREAM("Material unloaded");
      unload_res.unload_state = "Succeeded";
      material_unload_server_ptr_->setSucceeded(unload_res);
      setState(StateTypes::WAITING);
      break;



    case StateTypes::ABORTING:
      ROS_ERROR("Entering state machine abort sequence");
      setState(StateTypes::ABORT_GOALS);
      break;

    case StateTypes::ABORT_GOALS:
      ROS_INFO_STREAM("Aborting goals");
      abortActionServers();
      setState(StateTypes::CANCEL_REQUESTS);
      break;

    case StateTypes::CANCEL_REQUESTS:
      ROS_INFO_STREAM("Canceling requests");
      cancelActionClients();
      setState(StateTypes::ABORTED);
      break;

    case StateTypes::ABORTED:
      ROS_INFO_STREAM_THROTTLE(30, "Robot in ABORTED state");
      break;

    case StateTypes::RESETTING:
      setState(StateTypes::R_WAIT_FOR_HOME);
      break;

    case StateTypes::R_WAIT_FOR_HOME:
      if( isHome())
      {
        setState(StateTypes::R_SET_MAT_ACTIONS_NOT_READY);
      }
      else
      {
        ROS_ERROR_STREAM("Robot not in home state for FAULT RESET");
        setState(StateTypes::ABORTING);
      }
      break;

    case StateTypes::R_SET_MAT_ACTIONS_NOT_READY:
      setMatActionsNotReady();
      setState(StateTypes::IDLE);
      break;

    default:
      ROS_ERROR_STREAM("Unhandled state: " << state_ << ", setting state to aborted");
      setState(StateTypes::ABORTING);
      break;
  }
}

bool StateMachine::areActionsReady()
{
  return open_door_client_ptr_->isServerConnected() && close_door_client_ptr_->isServerConnected()
      && open_chuck_client_ptr_->isServerConnected() && close_chuck_client_ptr_->isServerConnected()
      && grasp_action_client_ptr_->isServerConnected() && vise_action_client_ptr_->isServerConnected()
      && joint_traj_client_ptr_->isServerConnected();
}

bool StateMachine::areServicesReady()
{
  return material_load_set_state_client_.exists() && material_unload_set_state_client_.exists()
      && trajectory_filter_client_.exists();
}

void StateMachine::abortActionServers()
{
  ROS_INFO_STREAM("Checking material load action server");
  if (material_load_server_ptr_->isActive())
  {
    MaterialLoadServer::Result res;
    res.load_state = "Failed";
    material_load_server_ptr_->setAborted(res);
    ROS_INFO_STREAM("Material Load goal aborted");
  }

  ROS_INFO_STREAM("Checking material unload action server");
  if (material_unload_server_ptr_->isActive())
  {
    MaterialUnloadServer::Result res;
    res.unload_state = "Failed";
    material_unload_server_ptr_->setAborted(res);
    ROS_INFO_STREAM("Material Unload goal aborted");
  }
}

void StateMachine::cancelActionClients()
{
  ROS_INFO_STREAM("Canceling all action clients");
  joint_traj_client_ptr_->cancelAllGoals();

  open_door_client_ptr_->cancelAllGoals();
  open_chuck_client_ptr_->cancelAllGoals();
  close_door_client_ptr_->cancelAllGoals();
  close_chuck_client_ptr_->cancelAllGoals();

  grasp_action_client_ptr_->cancelAllGoals();
  vise_action_client_ptr_->cancelAllGoals();
}

bool StateMachine::setMatActionsReady()
{
  return setMatLoad(mtconnect_msgs::SetMTConnectState::Request::READY)
      && setMatUnload(mtconnect_msgs::SetMTConnectState::Request::READY);
}

bool StateMachine::setMatActionsNotReady()
{
  return setMatLoad(mtconnect_msgs::SetMTConnectState::Request::NOT_READY)
      && setMatUnload(mtconnect_msgs::SetMTConnectState::Request::NOT_READY);
}

bool StateMachine::setMatLoad(int state)
{
  bool rtn = false;
  mat_load_set_state_.request.state_flag = state;

  if (material_load_set_state_client_.call(mat_load_set_state_))
  {
    if (mat_load_set_state_.response.accepted)
    {
      ROS_INFO_STREAM("Material load set to: " << state);
      rtn = true;
    }
    else
    {
      ROS_WARN_STREAM("Material load failed to set to: " << state);
      rtn = false;
    }
  }
  else
  {
    ROS_WARN_STREAM("Material load service call failed");
    rtn = false;
  }

  return rtn;
}

bool StateMachine::setMatUnload(int state)
{
  bool rtn = false;
  mat_unload_set_state_.request.state_flag = state;

  if (material_unload_set_state_client_.call(mat_unload_set_state_))
  {
    if (mat_unload_set_state_.response.accepted)
    {
      ROS_INFO_STREAM("Material unload set to: " << state);
      rtn = true;
    }
    else
    {
      ROS_WARN_STREAM("Material unload failed to set to: " << state);
      rtn = false;
    }
  }
  else
  {
    ROS_WARN_STREAM("Material unload service call failed");
    rtn = false;
  }

  return rtn;
}

void StateMachine::errorChecks()
{
  using namespace industrial_msgs;
  bool error = false;

  if (robot_status_msg_.e_stopped.val == TriState::TRUE)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "Robot estopped(" << robot_status_msg_.e_stopped << " aborting");
    error = true;
  }
  if (robot_status_msg_.in_error.val == TriState::TRUE)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "General robot error(" << robot_status_msg_.in_error << " aborting");
    error = true;
  }

  if (error && !(StateTypes::ABORTING >= state_ && state_ <= StateTypes::ABORTED))
  {
    ROS_INFO_STREAM("One or more general errors detected, aborting");
    setState(StateTypes::ABORTING);
  }
}

void StateMachine::overrideChecks()
{
  ros::NodeHandle ph("~");
  int state_override = StateTypes::INVALID;
  ph.getParamCached(PARAM_STATE_OVERRIDE, state_override);

  int force_fault_state = StateTypes::INVALID;
  ph.getParamCached(PARAM_FORCE_FAULT_STATE, force_fault_state);

  if (state_override != StateTypes::INVALID)
  {
    ROS_WARN_STREAM("Overriding state to: " << StateTypes::STATE_MAP[state_override]);
    setState(StateType(state_override));
    ph.setParam(PARAM_STATE_OVERRIDE, StateTypes::INVALID);
  }
  if (state_ == force_fault_state)
  {
    ROS_ERROR_STREAM("Forcing fault from state: "<< StateTypes::STATE_MAP[state_]);
    setState(StateTypes::ABORTING);
    ph.setParam(PARAM_FORCE_FAULT_STATE, StateTypes::INVALID);
  }
}

//Publishers
void StateMachine::callPublishers()
{
  robotStatusPublisher();
  robotSpindlePublisher();
  stateMachineStatusPublisher();
}
void StateMachine::robotStatusPublisher()
{
  using namespace industrial_msgs;
  // updating header time stamps
  robot_state_msg_.header.stamp = ros::Time::now();

  // TODO: May want to rethink these conditions, not
  if ((state_ != StateTypes::IDLE) && (state_ != StateTypes::ABORTED))
  {
    robot_state_msg_.avail.val = TriState::ENABLED;
    robot_state_msg_.mode.val = RobotMode::AUTO;
  }
  else
  {
    robot_state_msg_.avail.val = TriState::DISABLED;
    robot_state_msg_.mode.val = RobotMode::MANUAL;
  }

  //TODO: Figure out how the mode is supposed to be used
  //robot_state_msg_.mode.val = robot_status_msg_.mode.val;

  // TODO: Overriding these values until we know what they should be
  //robot_state_msg_.avail.val = TriState::ENABLED;
  //robot_state_msg_.mode.val = RobotMode::AUTO;
  robot_state_msg_.rexec.val = TriState::HIGH;

  // publishing
  robot_states_pub_.publish(robot_state_msg_);
}

void StateMachine::robotSpindlePublisher()
{
  using namespace industrial_msgs;
  robot_spindle_msg_.header.stamp = ros::Time::now();

  // TODO: Overriding these values until we know what they should be
  robot_spindle_msg_.c_unclamp.val = TriState::HIGH;
  robot_spindle_msg_.s_inter.val = TriState::HIGH;

  robot_spindle_pub_.publish(robot_spindle_msg_);
}


void StateMachine::stateMachineStatusPublisher()
{
  state_machine_stat_msg_.header.stamp = ros::Time::now();
  state_machine_stat_msg_.state = state_;
  state_machine_stat_msg_.state_name = StateTypes::STATE_MAP[state_];

  state_machine_pub_.publish(state_machine_stat_msg_);
}


//Callbacks
void StateMachine::materialLoadGoalCB(/*const MaterialLoadServer::GoalConstPtr &gh*/)
{
  switch (state_)
  {
    case StateTypes::WAITING:
      ROS_INFO_STREAM("Accepting material load request");
      material_load_server_ptr_->acceptNewGoal();
      //setMatUnload(MtConnectState::NOT_READY);
      setState(StateTypes::MATERIAL_LOADING);
      break;
    default:
      ROS_WARN_STREAM("Material load request received in wrong state: " << state_);
      MaterialLoadServer::Result res;
      res.load_state = "Failed";
      material_load_server_ptr_->acceptNewGoal();
      material_load_server_ptr_->setAborted(res);
      break;
  }
}
void StateMachine::materialUnloadGoalCB(/*const MaterialUnloadServer::GoalConstPtr &gh*/)
{
  switch (state_)
  {
    case StateTypes::WAITING:
      ROS_INFO_STREAM("Accepting material unload request");
      material_unload_server_ptr_->acceptNewGoal();
      //setMatLoad(MtConnectState::NOT_READY);
      setState(StateTypes::MATERIAL_UNLOADING);
      break;
    default:
      ROS_WARN_STREAM("Material unload request received in wrong state: " << state_);
      MaterialUnloadServer::Result res;
      res.unload_state = "Failed";
      material_unload_server_ptr_->acceptNewGoal();
      material_unload_server_ptr_->setAborted(res);
      break;
  }
}
void StateMachine::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  robot_status_msg_ = *msg;
}

void StateMachine::jointStatesCB(const sensor_msgs::JointStateConstPtr &msg)
{
  joint_state_msg_ = *msg;
}

bool StateMachine::externalCommandCB(mtconnect_example_msgs::StateMachineCmd::Request &req,
                                     mtconnect_example_msgs::StateMachineCmd::Response &res)
{
  using namespace mtconnect_example_msgs;

  switch (req.command)
  {
    case StateMachineCmd::Request::STOP:
      if (state_ == StateTypes::WAITING)
      {
        ROS_INFO_STREAM("External command STOP executing");
        setState(StateTypes::IDLE);
        res.accepted = true;
      }
      else
      {
        ROS_WARN_STREAM("External command STOP ignored, wrong state: " << state_);
        res.accepted = false;
      }
      break;

    case StateMachineCmd::Request::FAULT_RESET:
      if (state_ == StateTypes::ABORTED)
      {
        ROS_INFO_STREAM("External command FAULT_RESET executing");
        setState(StateTypes::RESETTING);
        res.accepted = true;
      }
      else
      {
        ROS_WARN_STREAM("External command FAULT_RESET ignored, wrong state: " << state_);
        res.accepted = false;
      }
      break;
    case StateMachineCmd::Request::START:
      if (state_ == StateTypes::IDLE)
      {
        ROS_INFO_STREAM("External command START executing");
        setState(StateTypes::INITING);
        res.accepted = true;
      }
      else
      {
        ROS_WARN_STREAM("External command START ignored, wrong state: " << state_);
        res.accepted = false;
      }
      break;

    default:
      ROS_WARN_STREAM("External command: ignored");
      res.accepted = false;
      break;
  }
  return true;
}

bool StateMachine::isActionComplete(int action_state)
{
  bool rtn = false;
  switch (action_state)
  {
    case actionlib::SimpleClientGoalState::PENDING:
      //ROS_INFO_STREAM_THROTTLE(1, "Action request is pending");
      break;

    case actionlib::SimpleClientGoalState::ACTIVE:
      //ROS_INFO_STREAM_THROTTLE(1, "Action request is active");
      break;

    case actionlib::SimpleClientGoalState::SUCCEEDED:
      rtn = true;
      break;

    case actionlib::SimpleClientGoalState::PREEMPTED:
    case actionlib::SimpleClientGoalState::LOST:
    case actionlib::SimpleClientGoalState::REJECTED:
    case actionlib::SimpleClientGoalState::ABORTED:
      // These states indicate something bad happened
      ROS_ERROR_STREAM("Bad action state: " << action_state);
      setState(StateTypes::ABORTING);
      break;

    default:
      ROS_ERROR_STREAM("Unrecognized action state: " << action_state);
      setState(StateTypes::ABORTING);
      break;
  }

  return rtn;
}

bool StateMachine::moveArm(const std::string & move_name)
{
  joint_traj_goal_.trajectory = *joint_paths_[move_name];
  //ROS_INFO_STREAM("Filtering a joint trajectory with " << joint_traj_goal_.trajectory.points.size() << "points");
  trajectory_filter_.request.trajectory = joint_traj_goal_.trajectory;
  if (!joint_traj_goal_.trajectory.points.empty())
  {
    if (trajectory_filter_client_.call(trajectory_filter_))
    {
      if (trajectory_filter_.response.error_code.val == trajectory_filter_.response.error_code.SUCCESS)
      {
        ROS_INFO_STREAM("======================== MOVING ROBOT ========================");
        ROS_INFO("Trajectory successfully filtered...sending goal");
        joint_traj_goal_.trajectory = trajectory_filter_.response.trajectory;
        ROS_INFO_STREAM("Sending a joint trajectory with " << joint_traj_goal_.trajectory.points.size() << "points");
        joint_traj_client_ptr_->sendGoal(joint_traj_goal_);
      }
      else
      {
        ROS_ERROR("Failed to process filter trajectory, entering fault state");
        setState(StateTypes::ABORTING);
      }
    }

    else
    {
      ROS_ERROR("Failed to call filter trajectory, entering fault state");
      setState(StateTypes::ABORTING);
    }
  }
  else
  {
    ROS_ERROR_STREAM(move_name << " trajectory is empty, failing");
    setState(StateTypes::ABORTING);
  }

  return true;
}

bool StateMachine::isMoveDone()
{
  return isActionComplete(joint_traj_client_ptr_->getState().state_);
}

void StateMachine::openDoor()
{
  ROS_INFO_STREAM("======================== OPENING DOOR ========================");
  mtconnect_msgs::OpenDoorGoal goal;
  goal.open_door = MTCONNECT_ACTION_ACTIVE_FLAG;
  open_door_client_ptr_->sendGoal(goal);
}

bool StateMachine::isDoorOpened()
{
  return isActionComplete(open_door_client_ptr_->getState().state_);
}

void StateMachine::closeDoor()
{
  ROS_INFO_STREAM("======================== CLOSING_DOOR ========================");
  mtconnect_msgs::CloseDoorGoal goal;
  goal.close_door = MTCONNECT_ACTION_ACTIVE_FLAG;
  close_door_client_ptr_->sendGoal(goal);
}

bool StateMachine::isDoorClosed()
{
  return isActionComplete(close_door_client_ptr_->getState().state_);
}

void StateMachine::openChuck()
{
  ROS_INFO_STREAM("======================== OPENING CHUCK ========================");
  // Actual chuck
  mtconnect_msgs::OpenChuckGoal chuck_goal;
  chuck_goal.open_chuck = MTCONNECT_ACTION_ACTIVE_FLAG;
  open_chuck_client_ptr_->sendGoal(chuck_goal);

  // Simulated chuck
  object_manipulation_msgs::GraspHandPostureExecutionGoal vise_goal;
  vise_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;
  vise_action_client_ptr_->sendGoal(vise_goal);
}

bool StateMachine::isChuckOpened()
{
  return isActionComplete(open_chuck_client_ptr_->getState().state_);
}

void StateMachine::closeChuck()
{
  ROS_INFO_STREAM("======================== CLOSING CHUCK ========================");
  // Actual chuck
  mtconnect_msgs::CloseChuckGoal chuck_goal;
  chuck_goal.close_chuck = MTCONNECT_ACTION_ACTIVE_FLAG;
  close_chuck_client_ptr_->sendGoal(chuck_goal);

  // Simulated chuck
  object_manipulation_msgs::GraspHandPostureExecutionGoal vise_goal;
  vise_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
  vise_action_client_ptr_->sendGoal(vise_goal);
}

bool StateMachine::isChuckClosed()
{
  return isActionComplete(close_chuck_client_ptr_->getState().state_)
      && isActionComplete(grasp_action_client_ptr_->getState().state_);
}

void StateMachine::openGripper()
{
  ROS_INFO_STREAM("======================== OPENING GRIPPER ========================");
  object_manipulation_msgs::GraspHandPostureExecutionGoal goal;
  goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;
  grasp_action_client_ptr_->sendGoal(goal);
}

bool StateMachine::isGripperOpened()
{
  return isActionComplete(grasp_action_client_ptr_->getState().state_);
}

void StateMachine::closeGripper()
{
  ROS_INFO_STREAM("======================== CLOSING GRIPPER ========================");
  object_manipulation_msgs::GraspHandPostureExecutionGoal goal;
  goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
  grasp_action_client_ptr_->sendGoal(goal);
}

bool StateMachine::isGripperClosed()
{
  return isActionComplete(grasp_action_client_ptr_->getState().state_);
}

bool StateMachine::isHome()
{
  bool rtn = false;

  if( home_check_ )
  {
    ROS_INFO_STREAM("Home checking ENABLED, returning range check");
    rtn = industrial_robot_client::utils::isWithinRange(home_->group_->joint_names_, home_->values_,
                                                      joint_state_msg_.name, joint_state_msg_.position, home_tol_);
  }
  else
  {
    ROS_WARN_STREAM("Home checking DISABLED, returning true");
    rtn = true;
  }

  return rtn;
}
