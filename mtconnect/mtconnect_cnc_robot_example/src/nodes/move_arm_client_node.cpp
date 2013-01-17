/*
 * move_arm_client_node.cpp
 *
 *  Created on: Jan 16, 2013
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <tf/tf.h>


// aliases
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
typedef boost::shared_ptr<MoveArmClient> MoveArmClientPtr;

// defaults
static const std::string DEFAULT_MOVE_ARM_ACTION = "move_arm_action";
static const std::string DEFAULT_PATH_PLANNER = "ompl_planning/plan_kinematic_path";
static const int DEFAULT_PATH_PLANNING_ATTEMPTS = 2;
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
				ss<<"\t   axis:[ "<<q.getAxis().getX()<<", "<<q.getAxis().getX()<<", "<<
						q.getAxis().getX()<<"], angle: "<< q.getAngle()<<"\n";
			}

			return ss.str();
		}

		bool parseParam(XmlRpc::XmlRpcValue &paramVal)
		{
			XmlRpc::XmlRpcValue cartesian_points_param;
			ros::NodeHandle nh;

			bool success;
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
					for(std::size_t i = 0; cartesian_points_param.size(); i++)
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
				}
			}
			else
			{
				ROS_ERROR_STREAM(ros::this_node::getName()<<": Parsing error in cartesian_trajectory parameter");
			}

			ROS_INFO_STREAM(ros::this_node::getName()<<": cartesian_trajectory parameter successfully parsed"<<toString());
			return success;
		}

		bool fetchParameters(std::string nameSpace= "")
		{
			XmlRpc::XmlRpcValue val;
			ros::NodeHandle nh;
			bool success = nh.getParam("cartesian_trajectory",val) && parseParam(val);
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
		goal.motion_plan_request.group_name = cartesian_traj_.arm_group_;
		goal.motion_plan_request.num_planning_attempts = DEFAULT_PATH_PLANNING_ATTEMPTS;
		goal.planner_service_name = DEFAULT_PATH_PLANNER;
		goal.motion_plan_request.planner_id = "";
		goal.motion_plan_request.allowed_planning_time = ros::Duration(DEFAULT_PATH_PLANNING_TIME);

		SimplePoseConstraint pose_constraint;
		pose_constraint.absolute_position_tolerance.x = 0.02f;
		pose_constraint.absolute_position_tolerance.y = 0.02f;
		pose_constraint.absolute_position_tolerance.z = 0.02f;
		pose_constraint.absolute_roll_tolerance = 0.04f;
		pose_constraint.absolute_pitch_tolerance = 0.04f;
		pose_constraint.absolute_yaw_tolerance = 0.04f;

		while(ros::ok())
		{
			// clearing goal
			goal.motion_plan_request.goal_constraints.position_constraints.clear();
			goal.motion_plan_request.goal_constraints.orientation_constraints.clear();
			goal.motion_plan_request.goal_constraints.joint_constraints.clear();
			goal.motion_plan_request.goal_constraints.visibility_constraints.clear();


			std::vector<tf::Transform>::iterator i;
			geometry_msgs::Pose pose;
			for(i = cartesian_traj_.cartesian_points_.begin(); i != cartesian_traj_.cartesian_points_.end();i++)
			{
				tf::poseTFToMsg(*i,pose);
				pose_constraint.pose = pose;
				arm_navigation_msgs::addGoalConstraintToMoveArmGoal(pose_constraint,goal);
			}

			// sending goal
			ROS_INFO_STREAM(ros::this_node::getName()<<": Sending Cartesian Goal");
			move_arm_client_ptr_->sendGoal(goal);
			if(move_arm_client_ptr_->waitForResult(ros::Duration(200.0f)))
			{
				ROS_INFO_STREAM(ros::this_node::getName()<<": Goal Achieved");
			}
			else
			{
				move_arm_client_ptr_->cancelGoal();
				actionlib::SimpleClientGoalState stateFlag = move_arm_client_ptr_->getState();
				ROS_ERROR_STREAM(ros::this_node::getName()<<": Goal Failed with error flag: "<<(unsigned int)stateFlag.state_);
			}
		}

	}

	bool fetchParameters(std::string nameSpace = "")
	{
		bool success = cartesian_traj_.fetchParameters("");
		return success;
	}

protected:

	bool setup()
	{
		ros::NodeHandle nh;
		bool success = true;

		if(!fetchParameters())
		{
			return false;
		}

		move_arm_client_ptr_ = MoveArmClientPtr(new MoveArmClient(DEFAULT_MOVE_ARM_ACTION,true));
		unsigned int attempts = 0;
		while(attempts++ < 20)
		{
			ROS_WARN_STREAM(ros::this_node::getName()<<": waiting for "<<DEFAULT_MOVE_ARM_ACTION<<" server");
			success = move_arm_client_ptr_->waitForResult(ros::Duration(5.0f));
			if(success)
			{
				ROS_INFO_STREAM(ros::this_node::getName()<<": Found "<<DEFAULT_MOVE_ARM_ACTION<<" server");
			}
		}

		return success;
	}

protected:

	// ros
	MoveArmClientPtr move_arm_client_ptr_;

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
