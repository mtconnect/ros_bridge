
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

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <object_manipulation_msgs/PlaceGoal.h>
#include <object_manipulation_msgs/PickupGoal.h>
#include <boost/tuple/tuple.hpp>
#include <sensor_msgs/JointState.h>

namespace move_arm_utils
{

bool parsePoint(XmlRpc::XmlRpcValue &val, geometry_msgs::Point &point);

bool parseOrientation(XmlRpc::XmlRpcValue &val, geometry_msgs::Quaternion &q);

bool parseOrientation(XmlRpc::XmlRpcValue &val, tf::Quaternion &q);

bool parseVect3(XmlRpc::XmlRpcValue &val,tf::Vector3 &v);

bool parseVect3(XmlRpc::XmlRpcValue &val, geometry_msgs::Vector3 &v);

bool parsePose(XmlRpc::XmlRpcValue &val, geometry_msgs::Pose &pose);

bool parseTransform(XmlRpc::XmlRpcValue &val, tf::Transform &t);

struct CartesianTrajectory
{
public:
	CartesianTrajectory():
		frame_id_("base_link"),
		cartesian_points_()
	{

	}

	~CartesianTrajectory()
	{

	}

	std::string toString();

	bool parseParameters(XmlRpc::XmlRpcValue &paramVal);

	void getMarker(nav_msgs::Path &p);

	bool fetchParameters(std::string nameSpace= "/cartesian_trajectory");

	std::string arm_group_;
	std::string frame_id_;
	std::string link_name_;
	std::vector<tf::Transform> cartesian_points_;
};

struct PickupGoalInfo : object_manipulation_msgs::PickupGoal
{
public:
	PickupGoalInfo()
	{

	}

	~PickupGoalInfo()
	{

	}

	bool fetchParameters(std::string nameSpace = "/pickup_goal");

	bool parseParameters(XmlRpc::XmlRpcValue &val);
};

struct PlaceGoalInfo : object_manipulation_msgs::PlaceGoal
{
public:
	PlaceGoalInfo()
	{

	}

	~PlaceGoalInfo()
	{

	}

	bool fetchParameters(std::string nameSpace = "/place_goal");

	bool parseParameters(XmlRpc::XmlRpcValue &val);

};

struct JointStateInfo: sensor_msgs::JointState
{
public:
	JointStateInfo():
		arm_group("")
	{

	}

	~JointStateInfo()
	{

	}

	bool fetchParameters(std::string nameSpace = "/joint_state");
	bool parseParameters(XmlRpc::XmlRpcValue &val);
	void toJointConstraints(double tol_above,double tol_below, std::vector<arm_navigation_msgs::JointConstraint> &joint_constraints);

	std::string arm_group;
};

}

#endif /* UTILITIES_H_ */
