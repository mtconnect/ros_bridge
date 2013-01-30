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
#include <nav_msgs/Path.h>
#include <object_manipulation_msgs/PlaceGoal.h>
#include <object_manipulation_msgs/PickupGoal.h>
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

	std::string toString();

	bool parseParam(XmlRpc::XmlRpcValue &paramVal);

	void getMarker(nav_msgs::Path &p);

	bool fetchParameters(std::string nameSpace= "");

	std::string arm_group_;
	std::string frame_id_;
	std::string link_name_;
	std::vector<tf::Transform> cartesian_points_;
};

struct PickPlaceMoveDetails
{
public:
	PickPlaceMoveDetails()
	{

	}

	~PickPlaceMoveDetails()
	{

	}

	bool parseParam(XmlRpc::XmlRpcValue &val);

	bool parsePickGoal(XmlRpc::XmlRpcValue &pickVal, object_manipulation_msgs::PickupGoal &g);

	bool parsePlaceGoal(XmlRpc::XmlRpcValue &val, object_manipulation_msgs::PlaceGoal &g);

	bool parsePose(XmlRpc::XmlRpcValue &poseVal, geometry_msgs::Pose &pose);

	bool parsePoint(XmlRpc::XmlRpcValue &pointVal, geometry_msgs::Point &point);

	bool parseVect3(XmlRpc::XmlRpcValue &vectVal,tf::Vector3 &v);

	bool parseVect3(XmlRpc::XmlRpcValue &val, geometry_msgs::Vector3 &v);

	bool fetchParameters(std::string nameSpace ="");

public:

	object_manipulation_msgs::PickupGoal pickup_goal_;
	object_manipulation_msgs::PlaceGoal place_goal_;

};

#endif /* UTILITIES_H_ */
