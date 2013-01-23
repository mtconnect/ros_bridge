/*
 * move_arm_client_node.cpp
 *
 *  Created on: Jan 16, 2013
 */
#include <ros/ros.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <boost/tuple/tuple.hpp>

// aliases
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
typedef boost::shared_ptr<MoveArmClient> MoveArmClientPtr;
typedef boost::shared_ptr<planning_environment::CollisionModels> CollisionModelsPtr;
typedef boost::shared_ptr<planning_models::KinematicState> KinematicStatePtr;

// defaults
static const std::string DEFAULT_PATH_MSG_TOPIC = "move_arm_path";
static const std::string DEFAULT_MOVE_ARM_ACTION = "move_arm_action";
static const std::string DEFAULT_PATH_PLANNER = "/ompl_planning/plan_kinematic_path";
static const std::string DEFAULT_PLANNING_SCENE_DIFF_SERVICE = "/environment_server/set_planning_scene_diff";
static const int DEFAULT_PATH_PLANNING_ATTEMPTS = 1;
static const double DEFAULT_PATH_PLANNING_TIME = 5.0f;

class MoveArmActionClient
{

public:
	struct CartesianTrajectory
	{
	public:
		CartesianTrajectory()
		{

		}

		~CartesianTrajectory()
		{

		}

		std::string toString()
		{
			std::stringstream ss;
			ss<<"\n";
			ss<<"arm_group: "<<arm_group_<<"\n";
			ss<<"frame_id: "<<frame_id_<<"\n";
			ss<<"link_name: "<<link_name_<<"\n";
			ss<<"via points: "<<"transform array with "<<cartesian_points_.size()<<" elements"<<"\n";
			tf::Vector3 pos;
			tf::Quaternion q;
			for(std::size_t i = 0; i < cartesian_points_.size(); i++)
			{
				tf::Transform &t = cartesian_points_[i];
				pos = t.getOrigin();
				q = t.getRotation();

				ss<<"\t - pos:[ "<<pos.getX()<<", "<<pos.getY()<<", "<<pos.getZ()<<"]\n";
				ss<<"\t   axis:[ "<<q.getAxis().getX()<<", "<<q.getAxis().getY()<<", "<<
						q.getAxis().getZ()<<"], angle: "<< q.getAngle()<<"\n";
			}

			return ss.str();
		}

		bool parseParam(XmlRpc::XmlRpcValue &paramVal)
		{
			XmlRpc::XmlRpcValue cartesian_points_param;
			ros::NodeHandle nh;

			bool success = true;
			if(paramVal.getType() == XmlRpc::XmlRpcValue::TypeStruct)
			{
				// fetching frame id and link name
				arm_group_ = static_cast<std::string>(paramVal["arm_group"]);
				frame_id_ = static_cast<std::string>(paramVal["frame_id"]);
				link_name_ = static_cast<std::string>(paramVal["link_name"]);

				// fetching cartesian via point struct array
				cartesian_points_param = paramVal["via_points"];

				// parsing each struct
				if((cartesian_points_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
						&& (cartesian_points_param[0].getType() == XmlRpc::XmlRpcValue::TypeStruct))
				{
					tf::Vector3 pos, axis;
					tf::Quaternion q;
					double val;
					XmlRpc::XmlRpcValue structElmt, posElmt, rotElmt, axisElmt;
					cartesian_points_.clear();

					ROS_INFO_STREAM(ros::this_node::getName()<<": parsing via points struct array"<<toString());
					for(std::size_t i = 0; i < cartesian_points_param.size(); i++)
					{
						structElmt = cartesian_points_param[i];
						posElmt = structElmt["position"];
						rotElmt = structElmt["orientation"];
						axisElmt = rotElmt["axis"];

						// fetching position
						val = static_cast<double>(posElmt["x"]);pos.setX(val);
						val = static_cast<double>(posElmt["y"]);pos.setY(val);
						val = static_cast<double>(posElmt["z"]);pos.setZ(val);

						// fetching rotation
						val = static_cast<double>(axisElmt["x"]);axis.setX(val);
						val = static_cast<double>(axisElmt["y"]);axis.setY(val);
						val = static_cast<double>(axisElmt["z"]);axis.setZ(val);
						val = static_cast<double>(rotElmt["angle"]);
						q.setRotation(axis,val);

						// storing as transform
						cartesian_points_.push_back(tf::Transform(q,pos));
					}
				}
				else
				{
					ROS_ERROR_STREAM(ros::this_node::getName()<<": Parsing error in cartesian_trajectory parameter");
					success = false;
				}
			}
			else
			{
				ROS_ERROR_STREAM(ros::this_node::getName()<<": Parsing error in cartesian_trajectory parameter");
				success = false;
			}

			ROS_INFO_STREAM(ros::this_node::getName()<<": cartesian_trajectory parameter successfully parsed"<<toString());
			return success;
		}

		void getMarker(nav_msgs::Path &p)
		{
			p.header.frame_id = frame_id_;
			p.header.stamp = ros::Time(0.0f);
			geometry_msgs::PoseStamped poseSt;

			// initializing pose stamped object
			poseSt.header.frame_id = frame_id_;
			poseSt.header.stamp = ros::Time(0.0f);

			std::vector<tf::Transform>::iterator i;
			for(i = cartesian_points_.begin(); i != cartesian_points_.end(); i++)
			{
				tf::poseTFToMsg(*i,poseSt.pose);
				p.poses.push_back(poseSt);
			}

		}

		bool fetchParameters(std::string nameSpace= "")
		{
			XmlRpc::XmlRpcValue val;
			ros::NodeHandle nh;
			bool success = nh.getParam(nameSpace + "/cartesian_trajectory",val) && parseParam(val);
			if(!success)
			{
				ROS_ERROR_STREAM(ros::this_node::getName()<<": Parsing error in cartesian_trajectory parameter");
			}
			return success;
		}

		std::string arm_group_;
		std::string frame_id_;
		std::string link_name_;
		std::vector<tf::Transform> cartesian_points_;
	};
public:
	MoveArmActionClient()
	:
		move_arm_client_ptr_()
	{

	}

	~MoveArmActionClient()
	{

	}

	void run()
	{
		using namespace arm_navigation_msgs;

		if(!setup())
		{
			return;
		}

		// creating move arm goal
		MoveArmGoal goal;
		goal.motion_plan_request.num_planning_attempts = DEFAULT_PATH_PLANNING_ATTEMPTS;
		goal.motion_plan_request.group_name = cartesian_traj_.arm_group_;
		goal.planner_service_name = DEFAULT_PATH_PLANNER;
		goal.motion_plan_request.planner_id = "";
		goal.motion_plan_request.allowed_planning_time = ros::Duration(DEFAULT_PATH_PLANNING_TIME);

		SimplePoseConstraint pose_constraint;
		pose_constraint.header.frame_id = cartesian_traj_.frame_id_;
		pose_constraint.link_name = cartesian_traj_.link_name_;
		pose_constraint.absolute_position_tolerance.x = 0.02f;
		pose_constraint.absolute_position_tolerance.y = 0.02f;
		pose_constraint.absolute_position_tolerance.z = 0.02f;
		pose_constraint.absolute_roll_tolerance = 0.04f;
		pose_constraint.absolute_pitch_tolerance = 0.04f;
		pose_constraint.absolute_yaw_tolerance = 0.04f;

		ros::AsyncSpinner spinner(2);
		spinner.start();

		while(ros::ok())
		{

			// proceeding if latest robot state can be retrieved
//			if(!getArmStartState(cartesian_traj_.arm_group_,goal.motion_plan_request.start_state))
//			{
//				continue;
//			}

			ROS_INFO_STREAM(ros::this_node::getName()<<": Sending Cartesian Goal with "<<cartesian_traj_.cartesian_points_.size()<<" via points");
			std::vector<tf::Transform>::iterator i;
			for(i = cartesian_traj_.cartesian_points_.begin(); i != cartesian_traj_.cartesian_points_.end();i++)
			{
				// clearing goal
				goal.motion_plan_request.goal_constraints.position_constraints.clear();
				goal.motion_plan_request.goal_constraints.orientation_constraints.clear();
				goal.motion_plan_request.goal_constraints.joint_constraints.clear();
				goal.motion_plan_request.goal_constraints.visibility_constraints.clear();

				tf::poseTFToMsg(*i,pose_constraint.pose);
				arm_navigation_msgs::addGoalConstraintToMoveArmGoal(pose_constraint,goal);
				//break;

				// sending goal
				move_arm_client_ptr_->sendGoal(goal);
				if(move_arm_client_ptr_->waitForResult(ros::Duration(200.0f)))
				{
					actionlib::SimpleClientGoalState stateFlag = move_arm_client_ptr_->getState();
					if(stateFlag.state_ == stateFlag.SUCCEEDED)
					{
						ROS_INFO_STREAM(ros::this_node::getName()<<": Goal Achieved");
					}
					else
					{
						ROS_ERROR_STREAM(ros::this_node::getName()<<": Goal Rejected with error flag: "<<(unsigned int)stateFlag.state_);
						break;
					}

				}
				else
				{
					move_arm_client_ptr_->cancelGoal();
					actionlib::SimpleClientGoalState stateFlag = move_arm_client_ptr_->getState();
					ROS_ERROR_STREAM(ros::this_node::getName()<<": Goal Failed with error flag: "<<(unsigned int)stateFlag.state_);
					break;
				}
			}

			//ros::Duration(2.0f).sleep();
		}

	}

	bool fetchParameters(std::string nameSpace = "")
	{
		bool success = cartesian_traj_.fetchParameters(nameSpace);
		return success;
	}

	void timerCallback(const ros::TimerEvent &evnt)
	{
		path_pub_.publish(path_msg_);
	}

protected:

	bool getArmStartState(std::string group_name, arm_navigation_msgs::RobotState &robot_state)
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

	bool setup()
	{
		ros::NodeHandle nh;
		bool success = true;

		if(!fetchParameters())
		{
			return false;
		}

		// setting up action client
		//std::string action_name = "move_" + cartesian_traj_.arm_group_;
		move_arm_client_ptr_ = MoveArmClientPtr(new MoveArmClient(DEFAULT_MOVE_ARM_ACTION,true));
		unsigned int attempts = 0;
		while(attempts++ < 20)
		{
			ROS_WARN_STREAM(ros::this_node::getName()<<": waiting for "<<DEFAULT_MOVE_ARM_ACTION<<" server");
			success = move_arm_client_ptr_->waitForServer(ros::Duration(5.0f));
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
		cartesian_traj_.getMarker(path_msg_);

		// setting up ros timers
		publish_timer_ = nh.createTimer(ros::Duration(2.0f),&MoveArmActionClient::timerCallback,this);

		// setting up planning environment members
		collision_models_ptr_ = CollisionModelsPtr(new planning_environment::CollisionModels("robot_description"));

		// listing joints in group
		const std::vector<std::string> &joint_names =
				collision_models_ptr_->getKinematicModel()->getModelGroup(cartesian_traj_.arm_group_)->getJointModelNames();

		std::vector<std::string>::const_iterator i;
		std::stringstream ss;
		ss<<"\nJoint Names in group '"<<cartesian_traj_.arm_group_<<"'";
		for(i = joint_names.begin(); i != joint_names.end() ; i++)
		{
			ss<<"\n\t"<<*i;
		}
		ROS_INFO_STREAM(ros::this_node::getName()<<ss.str());

		return success;
	}

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

	// planning environment
	CollisionModelsPtr collision_models_ptr_;
	KinematicStatePtr arm_kinematic_state_ptr_;

	// trajectory
	CartesianTrajectory cartesian_traj_;

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"move_arm_client_node");
	ros::NodeHandle nh;

	MoveArmActionClient client;
	client.run();

	return 0;
}
