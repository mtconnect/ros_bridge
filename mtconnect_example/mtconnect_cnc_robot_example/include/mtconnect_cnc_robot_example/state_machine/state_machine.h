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

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assign/list_inserter.hpp>
#include <ros/ros.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/tuple/tuple.hpp>
#include <mtconnect_cnc_robot_example/utilities/utilities.h>
#include <mtconnect_cnc_robot_example/move_arm_action_clients/MoveArmActionClient.h>
#include <mtconnect_cnc_robot_example/Command.h>
#include <boost/enable_shared_from_this.hpp>
#include <industrial_msgs/RobotStatus.h>
#include <mtconnect_msgs/CloseChuckAction.h>
#include <mtconnect_msgs/OpenChuckAction.h>
#include <mtconnect_msgs/CloseDoorAction.h>
#include <mtconnect_msgs/OpenDoorAction.h>
#include <mtconnect_msgs/MaterialLoadAction.h>
#include <mtconnect_msgs/MaterialUnloadAction.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <mtconnect_cnc_robot_example/utilities/utilities.h>
#include <mtconnect_cnc_robot_example/state_machine/state_machine_interface.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <mtconnect_msgs/RobotSpindle.h>
#include <mtconnect_msgs/RobotStates.h>
#include <mtconnect_msgs/SetMTConnectState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

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
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointTractoryClient;
typedef boost::shared_ptr<MoveArmClient> MoveArmClientPtr;
typedef boost::shared_ptr<MovePickupClient> MovePickupClientPtr;
typedef boost::shared_ptr<MovePlaceClient> MovePlaceClientPtr;
typedef boost::shared_ptr<CncOpenDoorClient> CncOpenDoorClientPtr;
typedef boost::shared_ptr<CncCloseDoorClient> CncCloseDoorClientPtr;
typedef boost::shared_ptr<CncOpenChuckClient> CncOpenChuckClientPtr;
typedef boost::shared_ptr<CncCloseChuckClient> CncCloseChuckClientPtr;
typedef boost::shared_ptr<MaterialLoadServer> MaterialLoadServerPtr;
typedef boost::shared_ptr<MaterialUnloadServer> MaterialUnloadServerPtr;
typedef boost::shared_ptr<GraspActionClient> GraspActionClientPtr;
typedef boost::shared_ptr<JointTractoryClient> JointTractoryClientPtr;
typedef std::vector<int> MaterialHandlingSequence;

namespace mtconnect_cnc_robot_example {	namespace state_machine	{

	namespace tasks
	{
		enum Task
		{
			NO_TASK = int(0),
			ROBOT_PLACE_WORKPIECE,
			ROBOT_PICKUP_MATERIAL,
			ROBOT_APPROACH_CNC,
			ROBOT_ENTER_CNC,
			ROBOT_MOVE_TO_CHUCK,
			ROBOT_RETREAT_FROM_CHUCK,
			ROBOT_EXIT_CNC,
			ROBOT_MOVE_HOME,
			ROBOT_MOVE_WAIT,
			ROBOT_CARTESIAN_MOVE,
			CNC_OPEN_DOOR,
			CNC_CLOSE_DOOR,
			CNC_OPEN_CHUCK,
			CNC_CLOSE_CHUCK,
			VISE_OPEN,
			VISE_CLOSE,
			GRIPPER_OPEN,
			GRIPPER_CLOSE,
			MATERIAL_LOAD_START,
			MATERIAL_LOAD_END,
			MATERIAL_UNLOAD_START,
			MATERIAL_UNLOAD_END,
			TEST_TASK_START,
			TEST_TASK_END,

			JM_HOME_TO_READY,
			JM_READY_TO_APPROACH,
			JM_APPROACH_TO_PICK,
			JM_PICK_TO_DOOR,
			JM_DOOR_TO_CHUCK,
                        JM_CHUCK_TO_READY,
			JM_READY_TO_DOOR,
			JM_PICK_TO_HOME
		};

		static std::map<int,std::string> TASK_MAP =
				boost::assign::map_list_of(NO_TASK,"NO TASK")
		(ROBOT_PLACE_WORKPIECE,"ROBOT_PLACE_WORKPIECE")
		(ROBOT_PICKUP_MATERIAL,"ROBOT_PICKUP_MATERIAL")
		(ROBOT_APPROACH_CNC,"ROBOT_APPROACH_CNC")
		(ROBOT_ENTER_CNC,"ROBOT_ENTER_CNC")
		(ROBOT_MOVE_TO_CHUCK,"ROBOT_MOVE_TO_CHUCK")
		(ROBOT_RETREAT_FROM_CHUCK,"ROBOT_RETREAT_FROM_CHUCK")
		(ROBOT_EXIT_CNC,"ROBOT_EXIT_CNC")
		(ROBOT_MOVE_HOME,"ROBOT_MOVE_HOME")
		(ROBOT_CARTESIAN_MOVE,"ROBOT_CARTESIAN_MOVE")
		(CNC_OPEN_DOOR,"CNC_OPEN_DOOR")
		(CNC_CLOSE_DOOR,"CNC_CLOSE_DOOR")
		(CNC_OPEN_CHUCK,"CNC_OPEN_CHUCK")
		(CNC_CLOSE_CHUCK,"CNC_CLOSE_CHUCK")
		(VISE_OPEN,"VISE_OPEN")
		(VISE_CLOSE,"VISE_CLOSE")
		(GRIPPER_OPEN,"GRIPPER_OPEN")
		(GRIPPER_CLOSE,"GRIPPER_CLOSE")
		(MATERIAL_LOAD_END,"MATERIAL_LOAD_END")
		(MATERIAL_UNLOAD_END,"MATERIAL_UNLOAD_END")
		(JM_HOME_TO_READY, "JM_HOME_TO_READY")
		(JM_READY_TO_APPROACH, "JM_READY_TO_APPROACH")
		(JM_APPROACH_TO_PICK, "JM_APPROACH_TO_PICK")
		(JM_PICK_TO_DOOR, "JM_PICK_TO_DOOR")
		(JM_DOOR_TO_CHUCK, "JM_DOOR_TO_CHUCK")
                (JM_CHUCK_TO_READY, "JM_CHUCK_TO_READY")
		(JM_READY_TO_DOOR, "JM_READY_TO_DOOR")
		(JM_PICK_TO_HOME, "JM_PICK_TO_HOME");
	}

	class StateMachine : public StateMachineInterface , public MoveArmActionClient
	{
	public:

		// callback types
		typedef boost::function<void (const actionlib::SimpleClientGoalState& ,const arm_navigation_msgs::MoveArmResultConstPtr& ) > MoveArmDoneCallback;
		typedef boost::function<void (const actionlib::SimpleClientGoalState& ,const  object_manipulation_msgs::PickupResultConstPtr& ) > MovePickupDoneCallback;
		typedef boost::function<void (const actionlib::SimpleClientGoalState& ,const  object_manipulation_msgs::PlaceResultConstPtr& ) > MovePlaceDoneCallback;
		typedef boost::function<void (const actionlib::SimpleClientGoalState& ,const  object_manipulation_msgs::GraspHandPostureExecutionResultConstPtr& ) > GraspActionDoneCallback;
		typedef boost::function<void (const actionlib::SimpleClientGoalState& ,const  mtconnect_msgs::OpenDoorResultConstPtr & ) > OpenDoorDoneCallback;
		typedef boost::function<void (const actionlib::SimpleClientGoalState& ,const  mtconnect_msgs::CloseDoorResultConstPtr & ) > CloseDoorDoneCallback;
		typedef boost::function<void (const actionlib::SimpleClientGoalState& ,const  mtconnect_msgs::OpenChuckResultConstPtr & ) > OpenChuckDoneCallback;
		typedef boost::function<void (const actionlib::SimpleClientGoalState& ,const  mtconnect_msgs::CloseChuckResultConstPtr & ) > CloseChuckDoneCallback;

	public:

		StateMachine();
		virtual ~StateMachine();

		virtual void run();

	protected:

		bool fetch_parameters(std::string name_space = "");
		bool fetchParameters(std::string name_space = ""){return true;} // override inherited from move arm client
		bool setup();

		// ros timer callbaks
		void publish_robot_topics_timercb(const ros::TimerEvent &evnt);

		// action goal callbacks
		void material_load_goalcb(/*const MaterialLoadServer::GoalConstPtr &gh*/);
		void material_unload_goalcb(/*const MaterialUnloadServer::GoalConstPtr &gh*/);

		// wrappers for sending a goal to a move arm server
		bool moveArm(const geometry_msgs::PoseArray &cartesian_poses)
		{
			MoveArmActionClient::moveArm(cartesian_poses,false);
		}

		bool moveArm(move_arm_utils::JointStateInfo &joint_info);
		bool moveArm(std::string & move_name);

		// material load/unload specific
		bool run_task(int task_id);
		bool run_next_task();

		// task test
		void print_task_list()
		{
			using namespace mtconnect_cnc_robot_example::state_machine::tasks;
			std::map<int,std::string>::iterator i;
			std::cout<<"\tTasks List\n";
			for(i = TASK_MAP.begin(); i != TASK_MAP.end(); i++)
			{
				std::cout<<"\t- ("<<i->first<<") \t"<< i->second<<"\n";
			}
		}

		// fault related methods
		void cancel_active_material_requests();
		void cancel_active_action_goals();
		void get_param_force_fault_flags();
		bool get_param_fault_on_task_check(int task_id);
		bool all_action_servers_connected();
		bool check_arm_at_position(sensor_msgs::JointState &joints, double tolerance);

		// subscriber callback
		void ros_status_subs_cb(const industrial_msgs::RobotStatusConstPtr &msg);

		// service callbacks
		bool external_command_cb(mtconnect_cnc_robot_example::Command::Request &req,
				mtconnect_cnc_robot_example::Command::Response &res);

		// transition actions
		virtual bool on_startup();
		virtual bool on_ready();
		virtual bool on_robot_reset();
		virtual bool on_material_load_started();
		virtual bool on_material_load_completed();
		virtual bool on_material_unload_started();
		virtual bool on_material_unload_completed();
		virtual bool on_cnc_reset();
		virtual bool on_peripherals_reset();
		virtual bool on_robot_fault();
		virtual bool on_cnc_fault();
		virtual bool on_gripper_fault();
		virtual bool on_robot_moving();
		virtual bool on_cnc_moving();
		virtual bool on_gripper_moving();
		virtual bool on_test_task_started();
		virtual bool on_test_task_completed();
		virtual bool on_display_tasks()
		{
			print_task_list();
			return true;
		}

	protected:
		// material load/unload task sequence
		std::vector<int> material_load_task_sequence_;
                std::vector<int> jm_material_load_task_sequence_;
		std::vector<int> material_unload_task_sequence_;
		std::vector<int> jm_material_unload_task_sequence_;
		std::vector<int> current_task_sequence_; // will take the value of the load or unload task sequence
		int current_task_index_;

		// test tasks
		int test_task_id_;

		// task definitions
		std::string task_desc_;
		std::map<std::string, trajectory_msgs::JointTrajectoryPtr> joint_paths_;
		bool use_task_desc_motion;  //if true, will use task defined motion (instead of planners)

		// action servers
		MaterialLoadServerPtr material_load_server_ptr_;
		MaterialUnloadServerPtr material_unload_server_ptr_;

		// action clients
		MovePickupClientPtr move_pickup_client_ptr_;
		MovePlaceClientPtr move_place_client_ptr_;
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

		// topic subscribers
		ros::Subscriber robot_status_sub_;

		// service servers
		ros::ServiceServer external_command_srv_;

		// server clients
		ros::ServiceClient material_load_set_state_client_;
		ros::ServiceClient material_unload_set_state_client_;
		ros::ServiceClient trajectory_filter_client_;



		// robot state messages
		mtconnect_msgs::RobotStates robot_state_msg_;
		mtconnect_msgs::RobotSpindle robot_spindle_msg_;

		// service req/res
		mtconnect_msgs::SetMTConnectState mat_load_set_state_;
		mtconnect_msgs::SetMTConnectState mat_unload_set_state_;
		arm_navigation_msgs::FilterJointTrajectoryWithConstraints trajectory_filter_;

		// timers
		ros::Timer robot_topics_timer_;

		// robot moves initialized from parameters
		move_arm_utils::CartesianTrajectory traj_arbitrary_move_;
		move_arm_utils::CartesianTrajectory traj_approach_cnc_;
		move_arm_utils::CartesianTrajectory traj_enter_cnc_;
		move_arm_utils::CartesianTrajectory traj_move_to_chuck_;
		move_arm_utils::CartesianTrajectory traj_retreat_from_chuck_;
		move_arm_utils::CartesianTrajectory traj_exit_cnc_;

		move_arm_utils::PickupGoalInfo material_load_pickup_goal_;
		move_arm_utils::PlaceGoalInfo material_load_place_goal_;
		move_arm_utils::PickupGoalInfo material_unload_pickup_goal_;
		move_arm_utils::PlaceGoalInfo material_unload_place_goal_;
		move_arm_utils::JointStateInfo joint_home_pos_;
		move_arm_utils::JointStateInfo joint_wait_pos_;

		// function handle placeholders
		MoveArmDoneCallback move_arm_done_cb_;
		MovePickupDoneCallback move_pickup_done_cb_;
		MovePlaceDoneCallback move_place_done_cb_;
		GraspActionDoneCallback grasp_action_done_cb_;
		OpenDoorDoneCallback open_door_done_cb_;
		CloseDoorDoneCallback close_door_done_cb_;
		OpenChuckDoneCallback open_chuck_done_cb_;
		CloseChuckDoneCallback close_chuck_done_cb_;

		// move arm members
		geometry_msgs::PoseArray cartesian_poses_;
		arm_navigation_msgs::MoveArmGoal move_arm_joint_goal_;

		// joint trajectory members
		control_msgs::FollowJointTrajectoryGoal joint_traj_goal_;

	};

}	}

#endif /* STATE_MACHINE_H_ */
