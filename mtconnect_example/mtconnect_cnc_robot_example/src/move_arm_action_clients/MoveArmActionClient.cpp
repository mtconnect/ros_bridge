
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

#include <mtconnect_cnc_robot_example/move_arm_action_clients/MoveArmActionClient.h>
#include <arm_navigation_msgs/utils.h>

using namespace mtconnect_cnc_robot_example;

MoveArmActionClient::MoveArmActionClient()
:
	move_arm_client_ptr_()
{

}

MoveArmActionClient::~MoveArmActionClient()
{

}

void MoveArmActionClient::run()
{
	using namespace arm_navigation_msgs;

	if(!setup())
	{
		return;
	}

	ros::AsyncSpinner spinner(2);
	spinner.start();

	// producing cartesian goals in term of arm base and tip link
	geometry_msgs::PoseArray cartesian_poses;
	if(!getTrajectoryInArmSpace(cartesian_traj_,cartesian_poses))
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Cartesian Trajectory could not be transformed to arm space");
		return;
	}

	while(ros::ok())
	{
		if(!moveArm(cartesian_poses))
		{
			ros::Duration(DURATION_LOOP_PAUSE).sleep();
		}
	}
}

/*
 * Takes and array of poses where each pose is the desired tip link pose described in terms of the arm base
 */
bool MoveArmActionClient::moveArm(const geometry_msgs::PoseArray &cartesian_poses, bool wait_for_completion)
{
	using namespace arm_navigation_msgs;

	ROS_INFO_STREAM(ros::this_node::getName()<<": Sending Cartesian Goal with "<<cartesian_poses.poses.size()<<" via points");
	bool success = !cartesian_poses.poses.empty();
	std::vector<geometry_msgs::Pose>::const_iterator i;
	for(i = cartesian_poses.poses.begin(); i != cartesian_poses.poses.end();i++)
	{
		// clearing goal
		move_arm_goal_.motion_plan_request.goal_constraints.position_constraints.clear();
		move_arm_goal_.motion_plan_request.goal_constraints.orientation_constraints.clear();
		move_arm_goal_.motion_plan_request.goal_constraints.joint_constraints.clear();
		move_arm_goal_.motion_plan_request.goal_constraints.visibility_constraints.clear();

		//tf::poseTFToMsg(*i,move_pose_constraint_.pose);
		move_pose_constraint_.pose = *i;
		arm_navigation_msgs::addGoalConstraintToMoveArmGoal(move_pose_constraint_,move_arm_goal_);

		// sending goal
		move_arm_client_ptr_->sendGoal(move_arm_goal_);

		if(!wait_for_completion)
		{
			return success;
		}

		success = move_arm_client_ptr_->waitForResult(ros::Duration(DURATION_WAIT_RESULT));
		if(success)
		{
			success = actionlib::SimpleClientGoalState::SUCCEEDED == move_arm_client_ptr_->getState().state_;
			if(success)
			{
				ROS_INFO_STREAM(ros::this_node::getName()<<": Goal Achieved");
			}
			else
			{
				ROS_ERROR_STREAM(ros::this_node::getName()<<": Goal Rejected with error flag: "
						<<(unsigned int)move_arm_client_ptr_->getState().state_);
				break;
			}
		}
		else
		{
			move_arm_client_ptr_->cancelGoal();
			ROS_ERROR_STREAM(ros::this_node::getName()<<": Goal Failed with error flag: "
					<<(unsigned int)move_arm_client_ptr_->getState().state_);
			break;
		}
	}

	return success;
}

bool MoveArmActionClient::fetchParameters(std::string nameSpace)
{
	ros::NodeHandle nh("~");

	bool success =  nh.getParam(PARAM_ARM_GROUP,arm_group_) && cartesian_traj_.fetchParameters();
	return success;
}

void MoveArmActionClient::timerCallback(const ros::TimerEvent &evnt)
{
	// updating msg
	cartesian_traj_.getMarker(path_msg_);
	if(!path_msg_.poses.empty())
	{
		path_pub_.publish(path_msg_);
	}
}

bool MoveArmActionClient::getArmStartState(std::string group_name, arm_navigation_msgs::RobotState &robot_state)
{
	using namespace arm_navigation_msgs;

	// calling set planning scene service
	SetPlanningSceneDiff::Request planning_scene_req;
	SetPlanningSceneDiff::Response planning_scene_res;
	if(planning_scene_client_.call(planning_scene_req,planning_scene_res))
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<": call to set planning scene succeeded");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": call to set planning scene failed");
		return false;
	}

	// updating robot kinematic state from planning scene
	planning_models::KinematicState *st = collision_models_ptr_->setPlanningScene(planning_scene_res.planning_scene);
	if(st == NULL)
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Kinematic State for arm could not be retrieved from planning scene");
		return false;
	}

	planning_environment::convertKinematicStateToRobotState(*st,
														  ros::Time::now(),
														  collision_models_ptr_->getWorldFrameId(),
														  robot_state);

	collision_models_ptr_->revertPlanningScene(st);
	return true;
}

bool MoveArmActionClient::setup()
{
	ros::NodeHandle nh;
	bool success = true;

	if(!fetchParameters())
	{
		return false;
	}

	// setting up action client
	move_arm_client_ptr_ = MoveArmClientPtr(new MoveArmClient(DEFAULT_MOVE_ARM_ACTION,true));
	unsigned int attempts = 0;
	while(attempts++ < MAX_WAIT_ATTEMPTS)
	{
		ROS_WARN_STREAM(ros::this_node::getName()<<": waiting for "<<DEFAULT_MOVE_ARM_ACTION<<" server");
		success = move_arm_client_ptr_->waitForServer(ros::Duration(DURATION_WAIT_SERVER));
		if(success)
		{
			ROS_INFO_STREAM(ros::this_node::getName()<<": Found "<<DEFAULT_MOVE_ARM_ACTION<<" server");
			break;
		}
	}

	// setting up service clients
	planning_scene_client_ = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(DEFAULT_PLANNING_SCENE_DIFF_SERVICE);

	// setting up ros publishers
	path_pub_ = nh.advertise<nav_msgs::Path>(DEFAULT_PATH_MSG_TOPIC,1);

	// setting up ros timers
	publish_timer_ = nh.createTimer(ros::Duration(DURATION_TIMER_CYCLE),&MoveArmActionClient::timerCallback,this);

	// obtaining arm info
	collision_models_ptr_ = CollisionModelsPtr(new planning_environment::CollisionModels("robot_description"));
	getArmInfo(collision_models_ptr_.get(),arm_group_,base_link_frame_id_,tip_link_frame_id_);

	// initializing move arm request members
	move_arm_goal_.motion_plan_request.group_name = arm_group_;
	move_arm_goal_.motion_plan_request.num_planning_attempts = DEFAULT_PATH_PLANNING_ATTEMPTS;
	move_arm_goal_.planner_service_name = DEFAULT_PATH_PLANNER;
	move_arm_goal_.motion_plan_request.planner_id = "";
	move_arm_goal_.motion_plan_request.allowed_planning_time = ros::Duration(DEFAULT_PATH_PLANNING_TIME);

	move_pose_constraint_.header.frame_id = base_link_frame_id_;
	move_pose_constraint_.link_name = tip_link_frame_id_;
	move_pose_constraint_.absolute_position_tolerance.x = DEFAULT_POSITION_TOLERANCE;
	move_pose_constraint_.absolute_position_tolerance.y = DEFAULT_POSITION_TOLERANCE;
	move_pose_constraint_.absolute_position_tolerance.z = DEFAULT_POSITION_TOLERANCE;
	move_pose_constraint_.absolute_roll_tolerance = DEFAULT_ORIENTATION_TOLERANCE;
	move_pose_constraint_.absolute_pitch_tolerance = DEFAULT_ORIENTATION_TOLERANCE;
	move_pose_constraint_.absolute_yaw_tolerance = DEFAULT_ORIENTATION_TOLERANCE;

	return success;
}

bool MoveArmActionClient::getArmInfo(const planning_environment::CollisionModels *models,
		const std::string &arm_group,std::string &base_link, std::string &tip_link)
{
	ros::NodeHandle nh;
	XmlRpc::XmlRpcValue group_param;
	std::stringstream ss;
	bool success = false;

	// finding arm base and tip links from group member in ros parameters
	if(nh.getParam(DEFAULT_PLANNING_GROUPS_PARAMETER,group_param))
	{
		for(int i = 0; i < group_param.size(); i++)
		{
			XmlRpc::XmlRpcValue &element = group_param[i];
			if((element.getType() == XmlRpc::XmlRpcValue::TypeStruct) &&
					element.hasMember(PARAM_GROUP_KEY) &&
					(arm_group.compare(static_cast<std::string>(element[PARAM_GROUP_KEY])) == 0) &&
					element.hasMember(PARAM_BASE_KEY) &&
					element.hasMember(PARAM_TIP_KEY))
			{
				base_link = static_cast<std::string>(element[PARAM_BASE_KEY]);
				tip_link = static_cast<std::string>(element[PARAM_TIP_KEY]);
				success = true;
				break;
			}
		}
	}
	else
	{
		ROS_ERROR_STREAM("Did not find parameter '"<<DEFAULT_PLANNING_GROUPS_PARAMETER<<"', could not get arm information");
		return false;
	}

	// obtaining arm joint info
	const planning_models::KinematicModel::JointModelGroup *joint_model_group =
					models->getKinematicModel()->getModelGroup(arm_group);

	// printing arm details
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	const std::vector<std::string> &link_names = joint_model_group->getGroupLinkNames();
	std::vector<std::string>::const_iterator i;
	ss<<"\nArm Base Link Frame: "<<base_link;
	ss<<"\nArm Tip Link Frame: "<<tip_link;
	ss<<"\nJoint Names in group '"<<arm_group<<"'";
	for(i = joint_names.begin(); i != joint_names.end() ; i++)
	{
		ss<<"\n\t"<<*i;
	}

	ss<<"\nLink Names in group '"<<arm_group<<"'";
	for(i = link_names.begin(); i != link_names.end() ; i++)
	{
		ss<<"\n\t"<<*i;
	}
	ROS_INFO_STREAM(ros::this_node::getName()<<ss.str());

	return success;
}

bool MoveArmActionClient::getPoseInArmSpace(
		const CartesianGoal &cartesian_goal,geometry_msgs::Pose &base_to_tip_pose)
{
	// declaring transforms
	static tf::StampedTransform base_to_parent_tf;
	static tf::StampedTransform tcp_to_tip_link_tf;
	tf::Transform base_to_tip_tf; // desired pose of tip link in base coordinates

	const std::string &parent_frame = boost::tuples::get<0>(cartesian_goal);
	const std::string &tcp_frame = boost::tuples::get<1>(cartesian_goal);
	const tf::Transform &parent_to_tcp_tf = boost::tuples::get<2>(cartesian_goal);

	// finding transforms
	try
	{
		tf_listener_.lookupTransform(base_link_frame_id_,parent_frame,ros::Time(0),base_to_parent_tf);
		tf_listener_.lookupTransform(tcp_frame,tip_link_frame_id_,ros::Time(0),tcp_to_tip_link_tf);
	}
	catch(tf::LookupException &e)
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Unable to lookup transforms");
		return false;
	}

	base_to_tip_tf = ((tf::Transform)base_to_parent_tf) * parent_to_tcp_tf * ((tf::Transform)tcp_to_tip_link_tf);
	tf::poseTFToMsg(base_to_tip_tf,base_to_tip_pose);

	return true;
}

bool MoveArmActionClient::getTrajectoryInArmSpace(
		const CartesianTrajectory &cartesian_traj,geometry_msgs::PoseArray &base_to_tip_poses)
{
	// declaring transforms
	static tf::StampedTransform base_to_parent_tf;
	static tf::StampedTransform tcp_to_tip_link_tf;
	tf::Transform base_to_tip_tf; // desired pose of tip link in base coordinates
	geometry_msgs::Pose base_to_tip_pose;

	// finding transforms
	try
	{
		tf_listener_.lookupTransform(base_link_frame_id_,cartesian_traj.frame_id_,ros::Time(0),base_to_parent_tf);
		tf_listener_.lookupTransform(cartesian_traj.link_name_,tip_link_frame_id_,ros::Time(0),tcp_to_tip_link_tf);
	}
	catch(tf::LookupException &e)
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Unable to lookup transforms");
		return false;
	}

	std::vector<tf::Transform>::const_iterator i;
	const std::vector<tf::Transform> &goal_array = cartesian_traj.cartesian_points_;
	for(i = goal_array.begin(); i != goal_array.end(); i++)
	{
		base_to_tip_tf = (static_cast<tf::Transform>(base_to_parent_tf)) * (*i) * (static_cast<tf::Transform>(tcp_to_tip_link_tf));
		tf::poseTFToMsg(base_to_tip_tf,base_to_tip_pose);
		base_to_tip_poses.poses.push_back(base_to_tip_pose);
	}

	return true;
}



