/*
 * utilities.cpp
 *
 *  Created on: Jan 30, 2013
 *      Author: ros developer 
 */

#include <mtconnect_cnc_robot_example/utilities/utilities.h>


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

bool CartesianTrajectory::parseParam(XmlRpc::XmlRpcValue &paramVal)
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
	ros::NodeHandle nh;
	bool success = nh.getParam(nameSpace + "/cartesian_trajectory",val) && parseParam(val);
	if(!success)
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": Parsing error in cartesian_trajectory parameter");
	}
	return success;
}


bool PickPlaceMoveDetails::parseParam(XmlRpc::XmlRpcValue &val)
{
	bool success = true;
	std::string arm_name = static_cast<std::string>(val["arm_group"]);
	pickup_goal_.arm_name = arm_name;
	place_goal_.arm_name = arm_name;

	// parsing pick and place goals info
	success =  parsePickGoal(val["pick_goal"],pickup_goal_)
			&& parsePlaceGoal(val["place_goal"],place_goal_);
	if(success)
	{
		place_goal_.grasp = pickup_goal_.desired_grasps[0];
	}
	return success;
}

bool PickPlaceMoveDetails::parsePickGoal(XmlRpc::XmlRpcValue &pickVal, object_manipulation_msgs::PickupGoal &g)
{
	bool success = true;

	// allocating grasp and model data
	g.desired_grasps.resize(1);
	g.target.potential_models.resize(1);

	// parsing distances
	g.lift.desired_distance = static_cast<double>(pickVal["lift_distance"]);
	g.desired_grasps[0].desired_approach_distance = static_cast<double>(pickVal["approach_distance"]);

	// parsing reference frame info
	g.target.reference_frame_id = static_cast<std::string>(pickVal["frame_id"]);
	g.lift.direction.header.frame_id = static_cast<std::string>(pickVal["tool_name"]);

	// parsing grasp pose and direction
	success =  parseVect3(pickVal["lift_direction"],g.lift.direction.vector)
			&& parsePose(pickVal["grasp_pose"],g.desired_grasps[0].grasp_pose)
			&& parsePose(pickVal["object_pose"],g.target.potential_models[0].pose.pose);

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

bool PickPlaceMoveDetails::parsePlaceGoal(XmlRpc::XmlRpcValue &val, object_manipulation_msgs::PlaceGoal &g)
{
	bool success = true;

	// allocating place pose
	g.place_locations.resize(1);

	// parsing distances
	g.approach.desired_distance = static_cast<double>(val["approach_distance"]);
	g.desired_retreat_distance = static_cast<double>(val["retreat_distance"]);

	// parsing reference frame info
	g.place_locations[0].header.frame_id = static_cast<std::string>(val["frame_id"]);
	g.approach.direction.header.frame_id = static_cast<std::string>(val["tool_name"]);

	success = parseVect3(val["approach_direction"],g.approach.direction.vector)
			&& parsePose(val["place_pose"],g.place_locations[0].pose);

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


bool PickPlaceMoveDetails::parsePose(XmlRpc::XmlRpcValue &poseVal, geometry_msgs::Pose &pose)
{
	double angle;
	tf::Quaternion q;
	tf::Vector3 axis;
	XmlRpc::XmlRpcValue orientVal;

	// parsing position
	parsePoint(poseVal["position"],pose.position);

	// parsing orientation
	orientVal = poseVal["orientation"];
	angle = static_cast<double>(orientVal["angle"]);
	parseVect3(orientVal["axis"],axis);
	q = tf::Quaternion(axis,angle);
	tf::quaternionTFToMsg(q,pose.orientation);

	return true;
}

bool PickPlaceMoveDetails::parsePoint(XmlRpc::XmlRpcValue &pointVal, geometry_msgs::Point &point)
{
	// parsing components
	point.x = static_cast<double>(pointVal["x"]);
	point.y = static_cast<double>(pointVal["y"]);
	point.z = static_cast<double>(pointVal["z"]);
	return true;
}

bool PickPlaceMoveDetails::parseVect3(XmlRpc::XmlRpcValue &vectVal,tf::Vector3 &v)
{
	double val;
	val = static_cast<double>(vectVal["x"]);v.setX(val);
	val = static_cast<double>(vectVal["y"]);v.setY(val);
	val = static_cast<double>(vectVal["z"]);v.setZ(val);
	return true;
}

bool PickPlaceMoveDetails::parseVect3(XmlRpc::XmlRpcValue &val, geometry_msgs::Vector3 &v)
{
	tf::Vector3 vect;
	parseVect3(val,vect);
	tf::vector3TFToMsg(vect,v);
	return true;
}

bool PickPlaceMoveDetails::fetchParameters(std::string nameSpace)
{
	ros::NodeHandle nh;
	bool success = true;
	XmlRpc::XmlRpcValue val;

	success = nh.getParam(nameSpace + "/pick_place_moves_info",val);
	if(success)
	{
		parseParam(val);
	}

	return success;
}

