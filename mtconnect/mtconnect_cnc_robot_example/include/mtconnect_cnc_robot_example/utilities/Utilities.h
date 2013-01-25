/*
 * Utilities.h
 *
 *  Created on: Jan 25, 2013
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/tuple/tuple.hpp>

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

	std::string toString()
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

#endif /* UTILITIES_H_ */
