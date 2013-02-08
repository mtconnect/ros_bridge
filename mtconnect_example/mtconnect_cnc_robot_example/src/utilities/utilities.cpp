
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

#include <mtconnect_cnc_robot_example/utilities/utilities.h>

using namespace move_arm_utils;


bool move_arm_utils::parsePoint(XmlRpc::XmlRpcValue &pointVal, geometry_msgs::Point &point)
{
	// parsing components
	point.x = static_cast<double>(pointVal["x"]);
	point.y = static_cast<double>(pointVal["y"]);
	point.z = static_cast<double>(pointVal["z"]);
	return true;
}

bool move_arm_utils::parseOrientation(XmlRpc::XmlRpcValue &val, tf::Quaternion &q)
{
	double angle;
	tf::Vector3 axis;

	// parsing orientation
	angle = static_cast<double>(val["angle"]);
	parseVect3(val["axis"],axis);
	q.setRotation(axis,angle);
	return true;
}

bool move_arm_utils::parseOrientation(XmlRpc::XmlRpcValue &val, geometry_msgs::Quaternion &q)
{
	tf::Quaternion q_tf;

	if(!parseOrientation(val,q_tf))
	{
		return false;
	}

	tf::quaternionTFToMsg(q_tf,q);
	return true;
}

bool move_arm_utils::parseVect3(XmlRpc::XmlRpcValue &val, geometry_msgs::Vector3 &v)
{
	tf::Vector3 vect;
	parseVect3(val,vect);
	tf::vector3TFToMsg(vect,v);
	return true;
}

bool move_arm_utils::parseVect3(XmlRpc::XmlRpcValue &vectVal,tf::Vector3 &v)
{
	double val;
	val = static_cast<double>(vectVal["x"]);v.setX(val);
	val = static_cast<double>(vectVal["y"]);v.setY(val);
	val = static_cast<double>(vectVal["z"]);v.setZ(val);
	return true;
}

bool move_arm_utils::parsePose(XmlRpc::XmlRpcValue &val, geometry_msgs::Pose &pose)
{
	// parsing position and orientation values
	if(parsePoint(val["position"],pose.position) &&
			parseOrientation(val["orientation"],pose.orientation))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool move_arm_utils::parseTransform(XmlRpc::XmlRpcValue &val, tf::Transform &t)
{
	geometry_msgs::Pose p;

	if(!parsePose(val,p))
	{
		return false;
	}

	tf::poseMsgToTF(p,t);
	return true;
}

std::string CartesianTrajectory::toString()
{
	std::stringstream ss;
	ss<<"\n";
	ss<<"arm_group: "<<arm_group_<<"\n";
	ss<<"frame_id: "<<frame_id_<<"\n";
	ss<<"tool_name: "<<link_name_<<"\n";
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

bool CartesianTrajectory::parseParameters(XmlRpc::XmlRpcValue &paramVal)
{
	XmlRpc::XmlRpcValue cartesian_points_param;
	ros::NodeHandle nh;

	bool success = true;
	if(paramVal.getType() == XmlRpc::XmlRpcValue::TypeStruct)
	{
		// fetching frame id and link name
		arm_group_ = static_cast<std::string>(paramVal["arm_group"]);
		frame_id_ = static_cast<std::string>(paramVal["frame_id"]);
		link_name_ = static_cast<std::string>(paramVal["tool_name"]);

		// fetching cartesian via point struct array
		cartesian_points_param = paramVal["via_points"];

		// parsing each struct
		if((cartesian_points_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
				&& (cartesian_points_param[0].getType() == XmlRpc::XmlRpcValue::TypeStruct))
		{
			tf::Transform t;

			ROS_INFO_STREAM("Parsing via points struct array"<<toString());
			cartesian_points_.clear();
			for(int i = 0; i < cartesian_points_param.size(); i++)
			{

				if(!parseTransform(cartesian_points_param[i],t))
				{
					// parsing failed, exiting loop
					ROS_ERROR_STREAM("Parsing error in cartesian_trajectory point");
					success = false;
					break;
				}
				// storing as transform
				cartesian_points_.push_back(t);
			}
		}
		else
		{
			ROS_ERROR_STREAM("Parsing error in cartesian_trajectory, structure array is not well formed");
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

void CartesianTrajectory::getMarker(nav_msgs::Path &p)
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

bool CartesianTrajectory::fetchParameters(std::string nameSpace)
{
	XmlRpc::XmlRpcValue val;
	bool success = ros::param::get(nameSpace,val) && parseParameters(val);
	if(!success)
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Parsing error in cartesian_trajectory parameter");
	}
	return success;
}

bool PickupGoalInfo::fetchParameters(std::string nameSpace)
{
	ros::NodeHandle nh;
	XmlRpc::XmlRpcValue val;

	if(nh.getParam(nameSpace,val) && parseParameters(val))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool PickupGoalInfo::parseParameters(XmlRpc::XmlRpcValue &val)
{
	bool success = true;

	// allocating grasp and model data
	this->desired_grasps.resize(1);
	this->target.potential_models.resize(1);

	// parsing arm group name
	this->arm_name = static_cast<std::string>(val["arm_group"]);

	// parsing distances
	this->lift.desired_distance = static_cast<double>(val["lift_distance"]);
	this->desired_grasps[0].desired_approach_distance = static_cast<double>(val["approach_distance"]);

	// parsing reference frame info
	this->target.reference_frame_id = static_cast<std::string>(val["frame_id"]);
	this->lift.direction.header.frame_id = static_cast<std::string>(val["tool_name"]);

	// parsing grasp pose and direction
	success =  parseVect3(val["lift_direction"],this->lift.direction.vector)
			&& parsePose(val["grasp_pose"],this->desired_grasps[0].grasp_pose)
			&& parsePose(val["object_pose"],this->target.potential_models[0].pose.pose);

	if(success)
	{
		ROS_INFO_STREAM("Pickup goal parameters found");
	}
	else
	{
		ROS_ERROR_STREAM("Pickup goal parameters not found");
	}

	return success;
}

bool PlaceGoalInfo::fetchParameters(std::string nameSpace)
{
	XmlRpc::XmlRpcValue val;
	return ros::param::get(nameSpace,val) && parseParameters(val);
}

bool PlaceGoalInfo::parseParameters(XmlRpc::XmlRpcValue &val)
{
	bool success = true;

	// allocating place pose
	this->place_locations.resize(1);

	// parsing arm group name
	this->arm_name = static_cast<std::string>(val["arm_group"]);

	// parsing distances
	this->approach.desired_distance = static_cast<double>(val["approach_distance"]);
	this->desired_retreat_distance = static_cast<double>(val["retreat_distance"]);

	// parsing reference frame info
	this->place_locations[0].header.frame_id = static_cast<std::string>(val["frame_id"]);
	this->approach.direction.header.frame_id = static_cast<std::string>(val["tool_name"]);

	success = parseVect3(val["approach_direction"],this->approach.direction.vector)
			&& parsePose(val["place_pose"],this->place_locations[0].pose) && parsePose(val["grasp_pose"],this->grasp.grasp_pose);

	if(success)
	{
		ROS_INFO_STREAM("Place goal parameters found");
	}
	else
	{
		ROS_ERROR_STREAM("Place goal parameters not found");
	}

	return success;
}

bool JointStateInfo::fetchParameters(std::string nameSpace)
{
	XmlRpc::XmlRpcValue val;
	return ros::param::get(nameSpace,val) && parseParameters(val);
}

bool JointStateInfo::parseParameters(XmlRpc::XmlRpcValue &param)
{
	// declaring member name strings
	const std::string position_mb = "position";
	const std::string name_mb = "name";
	const std::string velocity_mb = "velocity";
	const std::string effort_mb = "effort";

	// clearing arrays
	position.clear();
	name.clear();
	velocity.clear();
	effort.clear();

	if(param.getType()== XmlRpc::XmlRpcValue::TypeArray && param[0].getType() == XmlRpc::XmlRpcValue::TypeStruct &&
			param[0].hasMember(position_mb) && param[0].hasMember(name_mb))
	{
		for(int i = 0 ; i < param.size(); i++)
		{
			XmlRpc::XmlRpcValue &element = param[i];
			position.push_back(static_cast<double>(element[position_mb]));
			name.push_back(static_cast<std::string>(element[name_mb]));

			// optional entries ( Assigns 0  when member is not found )
			velocity.push_back(element.hasMember(velocity_mb) ? static_cast<double>(element[velocity_mb]) : 0 );
			effort.push_back(element.hasMember(effort_mb) ?  static_cast<double>(element[effort_mb]) : 0);
		}
	}
	else
	{
		return false;
	}

	return true;
}

void JointStateInfo::toJointConstraints(double tol_above,double tol_below,
		std::vector<arm_navigation_msgs::JointConstraint> &joint_constraints)
{
	arm_navigation_msgs::JointConstraint val;
	val.tolerance_above = tol_above;
	val.tolerance_below = tol_below;
	val.weight = 0.0f;

	for(std::size_t i ; i < name.size(); i++)
	{
		val.joint_name = name[i];
		val.position = position[i];
	}
}

