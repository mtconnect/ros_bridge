/*
 * MoveArmActionClient.h
 *
 *  Created on: Jan 25, 2013
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
#include <mtconnect_cnc_robot_example/utilities/Utilities.h>

// aliases
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
typedef boost::shared_ptr<MoveArmClient> MoveArmClientPtr;
typedef boost::shared_ptr<planning_environment::CollisionModels> CollisionModelsPtr;
typedef boost::shared_ptr<planning_models::KinematicState> KinematicStatePtr;
typedef boost::tuple<std::string,std::string,tf::Transform> CartesianGoal;

class MoveArmActionClient
{

public:
	MoveArmActionClient();

	virtual ~MoveArmActionClient();

	virtual void run();

	/*
	 * Takes and array of poses where each pose is the desired tip link pose described in terms of the arm base
	 */
	virtual bool moveArm(const geometry_msgs::PoseArray &cartesian_poses);

	virtual bool fetchParameters(std::string nameSpace = "");

	virtual void timerCallback(const ros::TimerEvent &evnt);

protected:

	virtual bool getArmStartState(std::string group_name, arm_navigation_msgs::RobotState &robot_state);

	virtual bool setup();

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

#endif /* MOVEARMACTIONCLIENT_H_ */
