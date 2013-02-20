
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

#include <mtconnect_cnc_robot_example/move_arm_action_clients/MovePickPlaceServer.h>
#include <boost/bind.hpp>

// aliases
typedef actionlib::SimpleClientGoalState GoalState;

using namespace mtconnect_cnc_robot_example;

MovePickPlaceServer::MovePickPlaceServer() :
	MoveArmActionClient(),
	pickup_gh_(),
	place_gh_()
{
	// TODO Auto-generated constructor stub

}

MovePickPlaceServer::~MovePickPlaceServer()
{
	// TODO Auto-generated destructor stub
}

void MovePickPlaceServer::run()
{
	if(!setup())
	{
		return;
	}

	ros::AsyncSpinner spinner(2);
	spinner.start();

	// starting servers;
	arm_pickup_server_ptr_->start();
	arm_place_server_ptr_->start();

	while(ros::ok())
	{
		ros::Duration(DURATION_LOOP_PAUSE).sleep();
	}
}

//void MovePickPlaceServer::start()
//{
//	if(!setup())
//	{
//		return;
//	}
//
//	ros::AsyncSpinner spinner(2);
//	spinner.start();
//
//	// starting servers;
//	arm_pickup_server_ptr_->start();
//	arm_place_server_ptr_->start();
//
//	while(ros::ok())
//	{
//		ros::Duration(DURATION_LOOP_PAUSE).sleep();
//	}
//}

bool MovePickPlaceServer::fetchParameters(std::string name_space)
{
	ros::NodeHandle nh("~");
	bool success =  nh.getParam(PARAM_ARM_GROUP,arm_group_);
	if(success)
	{
		ROS_INFO_STREAM("Successfully read setup parameters");
	}
	else
	{
		ROS_ERROR_STREAM("Failed to read setup parameters");
	}

	return success;
}

bool MovePickPlaceServer::setup()
{
	int wait_attempts = 0;
	ros::NodeHandle nh;

	if(!MoveArmActionClient::setup())
	{
		return false;
	}

	// setting up pickup server
	ROS_INFO_STREAM("Setting up pickup server");
	arm_pickup_server_ptr_  = MoveArmPickupServerPtr(new MoveArmPickupServer(nh,DEFAULT_PICKUP_ACTION,
			boost::bind(&MovePickPlaceServer::pickupGoalCallback,this, _1),
			boost::bind(&MovePickPlaceServer::pickupCancelCallback,this,_1),
			false));

	// setting up place server
	ROS_INFO_STREAM("Setting up place server");
	arm_place_server_ptr_  = MoveArmPlaceServerPtr(new MoveArmPlaceServer(nh,DEFAULT_PLACE_ACTION,
			boost::bind(&MovePickPlaceServer::placeGoalCallback,this, _1),
			boost::bind(&MovePickPlaceServer::placeCancelCallback,this,_1),
			false));

	// setting up grasp action client
	ROS_INFO_STREAM("Setting up grasp client");
	grasp_action_client_ptr_ = GraspActionClientPtr(new GraspActionClient(DEFAULT_GRASP_ACTION,true));
	while(!grasp_action_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER)))
	{
		ROS_WARN_STREAM("Waiting for grasp action server "<<DEFAULT_GRASP_ACTION);
		if(wait_attempts++ > MAX_WAIT_ATTEMPTS)
		{
			ROS_ERROR_STREAM("Grasp action service was not found");
			return false;
		}
	}

	return true;
}

bool MovePickPlaceServer::moveArmThroughPickSequence(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
	// declaring cartesian path and grasp moves
	geometry_msgs::PoseArray pick_pose_sequence;
	geometry_msgs::PoseArray temp_pick_sequence;
	GraspGoal grasp_goal;

	// initializing arm path and grasp
	createPickupMoveSequence(pickup_goal,pick_pose_sequence);
	grasp_goal.grasp = pickup_goal.desired_grasps[0]; // this might not be needed

	// moving arm to pre-grasp pose
	temp_pick_sequence.poses.assign(1,pick_pose_sequence.poses.front());
	if(moveArm(temp_pick_sequence))
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<": Pre-grasp Arm Move Achieved");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Pre-grasp Arm Move Failed");
		return false;
	}

	// requesting gripper pre-grasp move
	grasp_goal.goal = GraspGoal::PRE_GRASP;
	grasp_action_client_ptr_->sendGoal(grasp_goal);
	if(grasp_action_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_GRASP_RESULT)))
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<": Pre-graps Gripper Move Achieved");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Pre-grasp Gripper Move Rejected with error flag: "
								<<(unsigned int)grasp_action_client_ptr_->getState().state_);
		return false;
	}

	// moving arm to pick pose
	temp_pick_sequence.poses.assign(1,*(pick_pose_sequence.poses.begin() + 1));
	if(moveArm(temp_pick_sequence))
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<": Pick Arm Move Achieved");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Pick Arm Move Failed");
		return false;
	}

	// requesting gripper grasp move
	grasp_goal.goal = GraspGoal::GRASP;
	grasp_action_client_ptr_->sendGoal(grasp_goal);
	if(grasp_action_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_GRASP_RESULT)))
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<": Grasp Gripper Move Achieved");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Grasp Gripper Move Rejected with error flag: "
								<<(unsigned int)grasp_action_client_ptr_->getState().state_);
		return false;
	}

	// moving arm to lift pose
	temp_pick_sequence.poses.assign(1,pick_pose_sequence.poses.back());
	if(moveArm(temp_pick_sequence))
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<": Lift Arm Moves Achieved");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Lift Arm Move Failed");
		return false;
	}

	return true;
}

bool MovePickPlaceServer::moveArmThroughPlaceSequence(const object_manipulation_msgs::PlaceGoal &place_goal)
{
	// declaring cartesian path and grasp moves
	geometry_msgs::PoseArray place_pose_sequence;
	geometry_msgs::PoseArray temp_place_sequence;
	GraspGoal grasp_goal;

	// initializing arm path and grasp
	createPlaceMoveSequence(place_goal,place_pose_sequence);
	grasp_goal.grasp = place_goal.grasp; // this might not be needed

	// moving arm through approach and place poses
	temp_place_sequence.poses.assign(place_pose_sequence.poses.begin(), place_pose_sequence.poses.end()-1);
	if(moveArm(temp_place_sequence))
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<": Approach and Place Arm Moves Achieved");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Approach and Place Arm Moves Failed");
		return false;
	}

	// requesting gripper release move
	grasp_goal.goal = GraspGoal::RELEASE;
	grasp_action_client_ptr_->sendGoal(grasp_goal);
	if(grasp_action_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_GRASP_RESULT)))
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<": Release Gripper Move Achieved");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Release Gripper Move Rejected with error flag: "
								<<(unsigned int)grasp_action_client_ptr_->getState().state_);
		return false;
	}

	// moving arm to retreat pose
	temp_place_sequence.poses.assign(1,place_pose_sequence.poses.back());
	if(moveArm(temp_place_sequence))
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<": Retreat Arm Move Achieved");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Retreat Arm Move Failed");
		return false;
	}

	return true;
}

void MovePickPlaceServer::pickupGoalCallback(PickupGoalHandle gh)
{
	const object_manipulation_msgs::PickupGoal &goal = *(gh.getGoal());
	object_manipulation_msgs::PickupResult res;

	// comparing goal handles
	if(gh.getGoalStatus().status == GoalState::ACTIVE && pickup_gh_ == gh)
	{
		// goal already being handled, ignoring
		ROS_WARN_STREAM("Pickup goal is already being processed, ignoring request");
		return;
	}

	// canceling current goals first
	pickup_gh_.getGoalStatus().status == GoalState::ACTIVE ? pickupCancelCallback(pickup_gh_): (void)NULL ;
	place_gh_.getGoalStatus().status == GoalState::ACTIVE ? placeCancelCallback(place_gh_) :(void) NULL ;

	// storing goal
	pickup_gh_ = gh;
	gh.setAccepted("accepted");

	// processing goal
	if(moveArmThroughPickSequence(goal))
	{
		res.manipulation_result.value = res.manipulation_result.SUCCESS;
		gh.setSucceeded(res,"Succeeded");
	}
	else
	{
		res.manipulation_result.value = res.manipulation_result.FAILED;
		gh.setAborted(res,"Failed");
	}
}

void MovePickPlaceServer::pickupCancelCallback(PickupGoalHandle gh)
{
	object_manipulation_msgs::PickupResult res;

	// comparing goal handles
	if(pickup_gh_ == gh && gh.getGoalStatus().status == GoalState::ACTIVE)
	{
		// goal already being handled, ignoring

		// checking state of current move arm goal
		if(move_arm_client_ptr_->getState().state_ == GoalState::ACTIVE)
		{
			// cancel goal and send new one
			move_arm_client_ptr_->cancelGoal();
		}

		// checking state of grasp goal
		if(grasp_action_client_ptr_->getState().state_ == GoalState::ACTIVE)
		{
			grasp_action_client_ptr_->cancelGoal();
		}

		res.manipulation_result.value = res.manipulation_result.CANCELLED;
		gh.setCanceled(res,"Canceled");

	}
}

void MovePickPlaceServer::placeGoalCallback(PlaceGoalHandle gh)
{
	const object_manipulation_msgs::PlaceGoal &goal = *(gh.getGoal());
	object_manipulation_msgs::PlaceResult res;

	// comparing handles
	// comparing goal handles
	if(gh.getGoalStatus().status == GoalState::ACTIVE && place_gh_ == gh)
	{
		// goal already being handled, ignoring
		ROS_WARN_STREAM("Place goal is already being processed, ignoring request");
		return;
	}

	// canceling current goals first
	pickup_gh_.getGoalStatus().status == GoalState::ACTIVE ? pickupCancelCallback(pickup_gh_): (void)NULL ;
	place_gh_.getGoalStatus().status == GoalState::ACTIVE ? placeCancelCallback(place_gh_) : (void)NULL ;

	// storing goal
	place_gh_ = gh;
	gh.setAccepted("accepted");

	// processing goal
	if(moveArmThroughPlaceSequence(goal))
	{
		res.manipulation_result.value = res.manipulation_result.SUCCESS;
		gh.setSucceeded(res,"Succeeded");
	}
	else
	{
		res.manipulation_result.value = res.manipulation_result.FAILED;
		gh.setAborted(res,"Failed");
	}
}

void MovePickPlaceServer::placeCancelCallback(PlaceGoalHandle gh)
{
	object_manipulation_msgs::PlaceResult res;

	// comparing goal handles
	if(place_gh_ == gh && gh.getGoalStatus().status == GoalState::ACTIVE)
	{
		// goal already being handled, ignoring

		// checking state of current move arm goal
		if(move_arm_client_ptr_->getState().state_ == GoalState::ACTIVE)
		{
			// cancel goal and send new one
			move_arm_client_ptr_->cancelGoal();
		}

		// checking state of grasp goal
		if(grasp_action_client_ptr_->getState().state_ == GoalState::ACTIVE)
		{
			grasp_action_client_ptr_->cancelGoal();
		}

		res.manipulation_result.value = res.manipulation_result.CANCELLED;
		gh.setCanceled(res,"Canceled");
	}
}

bool MovePickPlaceServer::createPickupMoveSequence(const object_manipulation_msgs::PickupGoal &goal,
		geometry_msgs::PoseArray &pickup_pose_sequence)
{
	tf::Transform obj_to_tcp_tf, world_to_obj_tf, world_to_tcp_tf, tcp_to_pregrasp_tf,tcp_to_lift_tf;
	tf::Vector3 approach_dir;
	double approach_dist, lift_dist;
	const object_manipulation_msgs::Grasp &grasp = goal.desired_grasps[0];
	CartesianTrajectory traj;

	// initializing cartesian trajectory
	traj.arm_group_ = goal.arm_name;
	traj.frame_id_ = goal.target.reference_frame_id; // reference frame
	traj.link_name_ = goal.lift.direction.header.frame_id; // direction must be defined relative to tcp pose

	// initializing transforms
	tf::poseMsgToTF(goal.target.potential_models[0].pose.pose,world_to_obj_tf);
	tf::poseMsgToTF(grasp.grasp_pose,obj_to_tcp_tf);
	world_to_tcp_tf = world_to_obj_tf * obj_to_tcp_tf;

	// initializing vectors and distances
	approach_dist =grasp.desired_approach_distance;
	lift_dist = goal.lift.desired_distance;
	tf::vector3MsgToTF(goal.lift.direction.vector,approach_dir);
	approach_dir.normalize();

	// creating sequence
	tcp_to_pregrasp_tf = tf::Transform(tf::Quaternion::getIdentity(),(-approach_dist) * approach_dir);
	tcp_to_lift_tf = tf::Transform(tf::Quaternion::getIdentity(),(-lift_dist) * approach_dir);
	traj.cartesian_points_.push_back(world_to_tcp_tf * tcp_to_pregrasp_tf ); // pre-grasp pose
	traj.cartesian_points_.push_back(world_to_tcp_tf); // pick pose
	traj.cartesian_points_.push_back(world_to_tcp_tf * tcp_to_lift_tf); // lift pose

	return getTrajectoryInArmSpace(traj,pickup_pose_sequence);
}

bool MovePickPlaceServer::createPlaceMoveSequence(const object_manipulation_msgs::PlaceGoal &goal,
		geometry_msgs::PoseArray &place_pose_sequence)
{
	tf::Transform obj_to_tcp_tf, world_to_place_tf, world_to_tcp_tf, tcp_to_approach_tf,tcp_to_retreat_tf;
	tf::Vector3 approach_dir;
	double approach_dist, retreat_dist;
	const object_manipulation_msgs::Grasp &grasp = goal.grasp;
	CartesianTrajectory traj;

	// initializing cartesian trajectory
	traj.arm_group_ = goal.arm_name;
	traj.frame_id_ = goal.place_locations[0].header.frame_id; // reference frame
	traj.link_name_ = goal.approach.direction.header.frame_id; // direction must be defined relative to tcp pose

	// initializing transforms
	tf::poseMsgToTF(goal.place_locations[0].pose,world_to_place_tf);
	tf::poseMsgToTF(grasp.grasp_pose,obj_to_tcp_tf);
	world_to_tcp_tf = world_to_place_tf * obj_to_tcp_tf;

	// initializing vectors and distances
	approach_dist =goal.approach.desired_distance;
	retreat_dist = goal.desired_retreat_distance;
	tf::vector3MsgToTF(goal.approach.direction.vector,approach_dir);
	approach_dir.normalize();

	// creating sequence
	tcp_to_approach_tf = tf::Transform(tf::Quaternion::getIdentity(),(-approach_dist) * approach_dir);
	tcp_to_retreat_tf = tf::Transform(tf::Quaternion::getIdentity(),(-retreat_dist) * approach_dir);
	traj.cartesian_points_.push_back(world_to_tcp_tf * tcp_to_approach_tf ); // approach pose
	traj.cartesian_points_.push_back(world_to_tcp_tf); // place pose
	traj.cartesian_points_.push_back(world_to_tcp_tf * tcp_to_retreat_tf); // retreat pose

	return getTrajectoryInArmSpace(traj,place_pose_sequence);
}
