
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

#ifndef MOVEARMACTIONCLIENT_H_
#define MOVEARMACTIONCLIENT_H_

#include <ros/ros.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/tuple/tuple.hpp>
#include <mtconnect_cnc_robot_example/utilities/utilities.h>

using namespace move_arm_utils;

namespace mtconnect_cnc_robot_example
{
// aliases
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
typedef boost::shared_ptr<MoveArmClient> MoveArmClientPtr;
typedef boost::shared_ptr<planning_environment::CollisionModels> CollisionModelsPtr;
typedef boost::shared_ptr<planning_models::KinematicState> KinematicStatePtr;
typedef boost::tuple<std::string,std::string,tf::Transform> CartesianGoal;
class MoveArmActionClient;
typedef boost::shared_ptr<MoveArmActionClient> MoveArmActionClientPtr;

// defaults and constants
static const std::string DEFAULT_PATH_MSG_TOPIC = "move_arm_path";
static const std::string DEFAULT_MOVE_ARM_ACTION = "move_arm_action";
static const std::string DEFAULT_PATH_PLANNER = "/ompl_planning/plan_kinematic_path";
static const std::string DEFAULT_PLANNING_SCENE_DIFF_SERVICE = "/environment_server/set_planning_scene_diff";
static const std::string DEFAULT_PLANNING_GROUPS_PARAMETER = "/robot_description_planning/groups";
static const int DEFAULT_PATH_PLANNING_ATTEMPTS = 2;
static const double DEFAULT_PATH_PLANNING_TIME = 5.0f;
static const double DEFAULT_ORIENTATION_TOLERANCE = 0.02f; //radians
static const double DEFAULT_POSITION_TOLERANCE = 0.008f; // meters
static const double DURATION_LOOP_PAUSE = 4.0f; // seconds
static const double DURATION_WAIT_RESULT= 80.0f;
static const double DURATION_TIMER_CYCLE = 2.0f;
static const double DURATION_WAIT_SERVER = 5.0f;
static const int MAX_WAIT_ATTEMPTS = 20;

// ros parameters
static const std::string PARAM_ARM_GROUP = "arm_group";
static const std::string PARAM_GROUP_KEY = "name";
static const std::string PARAM_BASE_KEY = "base_link";
static const std::string PARAM_TIP_KEY = "tip_link";

class MoveArmActionClient
{
public:
	MoveArmActionClient();

	virtual ~MoveArmActionClient();

	virtual void run();

	/*
	 * Takes and array of poses where each pose is the desired tip link pose described in terms of the arm base
	 */
	virtual bool moveArm(const geometry_msgs::PoseArray &cartesian_poses,bool wait_for_completion = true);

	virtual bool fetchParameters(std::string nameSpace = "");

	virtual void timerCallback(const ros::TimerEvent &evnt);

protected:

	virtual bool getArmStartState(std::string group_name, arm_navigation_msgs::RobotState &robot_state);

	virtual bool setup();

	bool getArmInfo(const planning_environment::CollisionModels *models,
			const std::string &arm_group,std::string &base_link, std::string &tip_link);

	bool getPoseInArmSpace(const CartesianGoal &cartesian_goal,geometry_msgs::Pose &base_to_tip_pose);

	bool getTrajectoryInArmSpace(const CartesianTrajectory &cartesian_traj,geometry_msgs::PoseArray &base_to_tip_poses);

protected:

	// ros action clients
	MoveArmClientPtr move_arm_client_ptr_;

	// ros service clients
	ros::ServiceClient planning_scene_client_;

	// ros publishers
	ros::Publisher path_pub_;

	// ros timers
	ros::Timer publish_timer_;

	// ros messages
	nav_msgs::Path path_msg_;

	// tf listener
	tf::TransformListener tf_listener_;

	// arm info
	CollisionModelsPtr collision_models_ptr_;
	KinematicStatePtr arm_kinematic_state_ptr_;
	std::string base_link_frame_id_;
	std::string tip_link_frame_id_;

	// ros parameters
	CartesianTrajectory cartesian_traj_;
	std::string arm_group_;

	// move arm request members
	arm_navigation_msgs::MoveArmGoal move_arm_goal_;
	arm_navigation_msgs::SimplePoseConstraint move_pose_constraint_;

};

}
#endif /* MOVEARMACTIONCLIENT_H_ */
