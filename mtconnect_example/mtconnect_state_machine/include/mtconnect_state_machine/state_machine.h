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

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <string>
#include <map>

#include <boost/assign/list_of.hpp>
#include <boost/assign/list_inserter.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>

#include <mtconnect_msgs/CloseChuckAction.h>
#include <mtconnect_msgs/OpenChuckAction.h>
#include <mtconnect_msgs/CloseDoorAction.h>
#include <mtconnect_msgs/OpenDoorAction.h>

#include <mtconnect_msgs/MaterialLoadAction.h>
#include <mtconnect_msgs/MaterialUnloadAction.h>

#include <mtconnect_msgs/RobotSpindle.h>
#include <mtconnect_msgs/RobotStates.h>

#include <mtconnect_example_msgs/StateMachineCmd.h>
#include <mtconnect_example_msgs/StateMachineStatus.h>

#include <mtconnect_msgs/SetMTConnectState.h>

#include <mtconnect_task_parser/task.h>

#include <industrial_msgs/RobotStatus.h>

namespace mtconnect_state_machine
{

/**
 * \brief Enumeration of state machine states
 */
namespace StateTypes
{
enum StateType
{
  INVALID = 0, IDLE = 1,

  INITING = 100,
  CHECK_HOME, WAIT_FOR_ACTIONS, WAIT_FOR_SERVICES, SET_MAT_ACTIONS_READY,

  CYCLE_BEGIN = 2000,
  WAITING = 2001,
  MATERIAL_LOADING = 2100,
  ML_MOVE_PICK_APPROACH, ML_WAIT_MOVE_PICK_APPROACH, ML_MOVE_PICK,
  ML_WAIT_MOVE_PICK, ML_PICK, ML_WAIT_PICK, ML_MOVE_CHUCK, ML_WAIT_MOVE_CHUCK,
  ML_CLOSE_CHUCK, ML_WAIT_CLOSE_CHUCK, ML_RELEASE_PART, ML_WAIT_RELEASE_PART,
  ML_MOVE_DOOR, ML_WAIT_MOVE_DOOR, ML_MOVE_HOME, ML_WAIT_MOVE_HOME,
  MATERIAL_LOADED = 2199,


  MATERIAL_UNLOADING = 2200,
  MU_MOVE_DOOR, MU_WAIT_MOVE_DOOR, MU_MOVE_CHUCK, MU_WAIT_MOVE_CHUCK, MU_PICK_PART, MU_WAIT_PICK_PART,
  MU_OPEN_CHUCK, MU_WAIT_OPEN_CHUCK, MU_MOVE_DROP, MU_WAIT_MOVE_DROP, MU_DROP, MU_WAIT_DROP,
  MU_MOVE_HOME, MU_WAIT_MOVE_HOME,
  MATERIAL_UNLOADED = 2299,
  CYCLE_END = 2999,

  STOPPING = 300,
  S_SET_MAT_ACTIONS_NOT_READY,
  STOPPED = 299,

  ABORTING = 500,
  ABORT_GOALS, CANCEL_REQUESTS,
  ABORTED = 599,

  RESETTING = 700,
  R_WAIT_FOR_HOME, R_SET_MAT_ACTIONS_NOT_READY,
  RESET = 799,

};
/**
 * \brief Map of states to state strings
 */
static std::map<int, std::string> STATE_MAP =
    boost::assign::map_list_of(INVALID, "INVALID")(IDLE, "IDLE")
    (INITING, "INITING")
    (CHECK_HOME, "CHECK_HOME")
    (WAIT_FOR_ACTIONS, "WAIT_FOR_ACTIONS")
    (WAIT_FOR_SERVICES, "WAIT_FOR_SERVICES")
    (SET_MAT_ACTIONS_READY,"SET_MAT_ACTIONS_READY")
    (WAITING, "WAITING")

    (MATERIAL_LOADING, "MATERIAL_LOADING")
    (ML_MOVE_PICK_APPROACH,"ML_MOVE_PICK_APPROACH")
    (ML_WAIT_MOVE_PICK_APPROACH,"ML_WAIT_MOVE_PICK_APPROACH")
    (ML_MOVE_PICK,"ML_MOVE_PICK")
    (ML_WAIT_MOVE_PICK,"ML_WAIT_MOVE_PICK")
    (ML_PICK,"ML_PICK")
    (ML_WAIT_PICK,"ML_WAIT_PICK")
    (ML_MOVE_CHUCK,"ML_MOVE_CHUCK")
    (ML_WAIT_MOVE_CHUCK,"ML_WAIT_MOVE_CHUCK")
    (ML_CLOSE_CHUCK,"ML_CLOSE_CHUCK")
    (ML_WAIT_CLOSE_CHUCK, "ML_WAIT_CLOSE_CHUCK")
    (ML_RELEASE_PART, "ML_RELEASE_PART")
    (ML_WAIT_RELEASE_PART, "ML_WAIT_RELEASE_PART")
    (ML_MOVE_DOOR, "ML_MOVE_DOOR")
    (ML_WAIT_MOVE_DOOR, "ML_WAIT_MOVE_DOOR")
    (ML_MOVE_HOME, "ML_MOVE_HOME")
    (ML_WAIT_MOVE_HOME, "ML_WAIT_MOVE_HOME")
    (MATERIAL_LOADED, "MATERIAL_LOADED")

    (MATERIAL_UNLOADING, "MATERIAL_UNLOADING")
    (MU_MOVE_DOOR, "MU_MOVE_DOOR")
    (MU_WAIT_MOVE_DOOR, "MU_WAIT_MOVE_DOOR")
    (MU_MOVE_CHUCK, "MU_MOVE_CHUCK")
    (MU_WAIT_MOVE_CHUCK, "MU_WAIT_MOVE_CHUCK")
    (MU_PICK_PART, "MU_PICK_PART")
    (MU_WAIT_PICK_PART, "MU_WAIT_PICK_PART")
    (MU_OPEN_CHUCK, "MU_OPEN_CHUCK")
    (MU_WAIT_OPEN_CHUCK, "MU_WAIT_OPEN_CHUCK")
    (MU_MOVE_DROP, "MU_MOVE_DROP")
    (MU_WAIT_MOVE_DROP, "MU_WAIT_MOVE_DROP")
    (MU_DROP, "MU_DROP")
    (MU_WAIT_DROP, "MU_WAIT_DROP")
    (MU_MOVE_HOME, "MU_MOVE_HOME")
    (MU_WAIT_MOVE_HOME, "MU_WAIT_MOVE_HOME")
    (MATERIAL_UNLOADED, "MATERIAL_UNLOADED")

    (STOPPING, "STOPPING")
    (S_SET_MAT_ACTIONS_NOT_READY, "S_SET_MAT_ACTIONS_NOT_READY")
    (STOPPED, "STOPPED")

    (ABORTING, "ABORTING")
    (ABORT_GOALS, "ABORT_GOALS")(CANCEL_REQUESTS, "CANCEL_REQUESTS")
    (ABORTED, "ABORTED")

    (RESETTING, "RESETTING")
    (R_WAIT_FOR_HOME, "R_WAIT_FOR_HOME")(R_SET_MAT_ACTIONS_NOT_READY, "SET_MAT_ACTIONS_NOT_READY")
    (RESET, "RESETED");

}
typedef StateTypes::StateType StateType;

// Typedefs
typedef actionlib::SimpleActionClient<mtconnect_msgs::OpenDoorAction> CncOpenDoorClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseDoorAction> CncCloseDoorClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::OpenChuckAction> CncOpenChuckClient;
typedef actionlib::SimpleActionClient<mtconnect_msgs::CloseChuckAction> CncCloseChuckClient;
typedef actionlib::SimpleActionServer<mtconnect_msgs::MaterialLoadAction> MaterialLoadServer;
typedef actionlib::SimpleActionServer<mtconnect_msgs::MaterialUnloadAction> MaterialUnloadServer;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointTractoryClient;

typedef boost::shared_ptr<CncOpenDoorClient> CncOpenDoorClientPtr;
typedef boost::shared_ptr<CncCloseDoorClient> CncCloseDoorClientPtr;
typedef boost::shared_ptr<CncOpenChuckClient> CncOpenChuckClientPtr;
typedef boost::shared_ptr<CncCloseChuckClient> CncCloseChuckClientPtr;
typedef boost::shared_ptr<MaterialLoadServer> MaterialLoadServerPtr;
typedef boost::shared_ptr<MaterialUnloadServer> MaterialUnloadServerPtr;
typedef boost::shared_ptr<GraspActionClient> GraspActionClientPtr;
typedef boost::shared_ptr<JointTractoryClient> JointTractoryClientPtr;

class StateMachine
{
public:

///////General state
  /**
   * \brief Default constructor (must call init function)
   *
   */
  StateMachine();

  /**
   * \brief Deconstructor
   *
   */
  ~StateMachine();

  /**
   * \brief Initializes statemachine, ros node, servers, and clients
   *
   * \return true on success, false otherwise.
   */
  bool init();

  /**
   * \brief Execute state machine (blocks permanently)
   *
   * Executes the state machine.  Handles all transitions
   *
   */
  void run();

  /**
   * \brief Execute state machine once
   *
   *
   */
  void runOnce();

protected:

///////General state

  void setState(StateType state)
  {
    ROS_INFO_STREAM("Changing state from: " << StateTypes::STATE_MAP[state_] << "(" << state_ << ")"
    " to " << StateTypes::STATE_MAP[state] << "(" << state << ")");
    state_ = state;
  }
  ;

  StateType getState()
  {
    return state_;
  }
  ;

//////MTConnect specific
  /**
   * \brief Internal node handle
   *
   */
  ros::NodeHandle nh_;

// Callbacks
  void materialLoadGoalCB(/*const MaterialLoadServer::GoalConstPtr &gh*/);
  void materialUnloadGoalCB(/*const MaterialUnloadServer::GoalConstPtr &gh*/);
  void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg);
  void jointStatesCB(const sensor_msgs::JointStateConstPtr &msg);
  bool externalCommandCB(mtconnect_example_msgs::StateMachineCmd::Request &req,
                         mtconnect_example_msgs::StateMachineCmd::Response &res);
  /**
   * \brief Checks remote action servers to see if they are ready
   *
   * \return true if all ready, otherwise false
   *
   */
  bool areActionsReady();

  /**
   * \brief Checks remote services to see if they are ready
   *
   * \return true if all ready, otherwise false
   *
   */
  bool areServicesReady();

  /**
   * \brief Aborts any server with active client requests
   *
   */
  void abortActionServers();

  /**
   * \brief Cancels any requests to action servers
   *
   */
  void cancelActionClients();

  /**
   * \brief Checks remote services to see if they are ready
   *
   * \return true if all ready, otherwise false
   *
   */
  bool setMatActionsReady();

  /**
   * \brief Checks remote services to see if they are ready
   *
   * \return true if all ready, otherwise false
   *
   */
  bool setMatActionsNotReady();

  /**
   * \brief Checks remote services to see if they are ready
   *
   * \return true if all ready, otherwise false
   *
   */
  bool setMatLoad(int state);
  bool setMatUnload(int state);

  // Publish methods

  void callPublishers();
  void robotStatusPublisher();
  void robotSpindlePublisher();
  void stateMachineStatusPublisher();

  // Action wrappers

  bool isActionComplete(int action_state);
  bool moveArm(const std::string & move_name);
  bool isMoveDone();

  void openDoor();
  bool isDoorOpened();
  void closeDoor();
  bool isDoorClosed();

  void openChuck();
  bool isChuckOpened();
  void closeChuck();
  bool isChuckClosed();

  void openGripper();
  bool isGripperOpened();
  void closeGripper();
  bool isGripperClosed();

  // Error checking
  void errorChecks();

  // Override checking
  void overrideChecks();

  // Home checking
  bool isHome();

private:

///////General state
  /**
   * \brief Holds current state
   *
   */
  StateType state_;

  /**
   * \brief Main loop rate
   *
   */
  int loop_rate_;

  /**
   * \brief enables/disables home checking
   *
   */
  bool home_check_;

  /**
   * \brief home position tolerance
   *
   */
  double home_tol_;
  /**
   * \brief cycle stop request flat
   *
   */
  bool cycle_stop_req_;

  /**
   * \brief material state (true if material is present)
   *
   */
  bool material_state_;

  /**
     * \brief internal flag to track material load state (should be encapsulated in material load)
     *
     */
  int material_load_state_;
//////MTConnect specific

  std::map<std::string, trajectory_msgs::JointTrajectoryPtr> joint_paths_;
  boost::shared_ptr<mtconnect::JointPoint> home_;

// action servers
  MaterialLoadServerPtr material_load_server_ptr_;
  MaterialUnloadServerPtr material_unload_server_ptr_;

// action clients
  CncOpenDoorClientPtr open_door_client_ptr_;
  CncCloseDoorClientPtr close_door_client_ptr_;
  CncOpenChuckClientPtr open_chuck_client_ptr_;
  CncCloseChuckClientPtr close_chuck_client_ptr_;
  GraspActionClientPtr grasp_action_client_ptr_;
  GraspActionClientPtr vise_action_client_ptr_;
  JointTractoryClientPtr joint_traj_client_ptr_;

// topic publishers (ros bridge components wait for these topics)
  ros::Publisher robot_states_pub_;
  ros::Publisher robot_spindle_pub_;
  ros::Publisher state_machine_pub_;

// topic subscribers
  ros::Subscriber robot_status_sub_;
  ros::Subscriber joint_states_sub_;

// service servers
  ros::ServiceServer external_command_srv_;

// server clients
  ros::ServiceClient material_load_set_state_client_;
  ros::ServiceClient material_unload_set_state_client_;
  ros::ServiceClient trajectory_filter_client_;

// pub messages
  mtconnect_msgs::RobotStates robot_state_msg_;
  mtconnect_msgs::RobotSpindle robot_spindle_msg_;
  mtconnect_example_msgs::StateMachineStatus state_machine_stat_msg_;

// sub messages
  sensor_msgs::JointState joint_state_msg_;
  industrial_msgs::RobotStatus robot_status_msg_;

// TODO: May remove these at some point
// service req/res
  mtconnect_msgs::SetMTConnectState mat_load_set_state_;
  mtconnect_msgs::SetMTConnectState mat_unload_set_state_;
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints trajectory_filter_;

  //action goals
  control_msgs::FollowJointTrajectoryGoal joint_traj_goal_;

};

} //mtconnect_state_machine

#endif /* STATE_MACHINE_H_ */
