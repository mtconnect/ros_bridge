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

#include <ros/ros.h>
#include <mtconnect_state_machine/utilities.h>
#include <boost/shared_ptr.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointTractoryClient;
typedef boost::shared_ptr<JointTractoryClient> JointTractoryClientPtr;
typedef std::map<std::string, trajectory_msgs::JointTrajectoryPtr>::iterator JTPMapItr;

static const std::string PARAM_TASK_DESCRIPTION = "task_description";
static const std::string DEFAULT_TRAJECTORY_FILTER_SERVICE = "filter_trajectory_with_constraints";
static const std::string DEFAULT_JOINT_TRAJ_ACTION = "joint_trajectory_action";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mtconnect_state_machine");

  ros::NodeHandle ph("~");
  ros::NodeHandle nh;

  std::string task_desc;
  std::map<std::string, trajectory_msgs::JointTrajectoryPtr> joint_paths;

  JointTractoryClientPtr joint_traj_client_ptr;
  control_msgs::FollowJointTrajectoryGoal joint_traj_goal;

  ros::ServiceClient trajectory_filter_client;
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints trajectory_filter_;

  // Load parameters

  ROS_INFO_STREAM("Loading parameters");
  if (!ph.getParam(PARAM_TASK_DESCRIPTION, task_desc))
  {
    ROS_ERROR("Failed to load task description parameter");
    return false;
  }

  std::map<std::string, boost::shared_ptr<mtconnect::JointPoint> > temp_pts;
  if (!mtconnect_state_machine::parseTaskXml(task_desc, joint_paths, temp_pts))
  {
    ROS_ERROR("Failed to parse xml");
    return false;
  }

  ROS_INFO_STREAM("Loaded " << joint_paths.size() << " paths");

  // Load actions/clients

  joint_traj_client_ptr = JointTractoryClientPtr(new JointTractoryClient(DEFAULT_JOINT_TRAJ_ACTION, true));

  trajectory_filter_client = nh.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(
      DEFAULT_TRAJECTORY_FILTER_SERVICE);

  ROS_INFO_STREAM("Waiting for joint trajectory filter and action server");
  joint_traj_client_ptr->waitForServer(ros::Duration(0));
  trajectory_filter_client.waitForExistence();

  ROS_INFO_STREAM("Starting loop through trajectory");

  while (ros::ok())
  {
    for(JTPMapItr itr = joint_paths.begin(); itr != joint_paths.end(); itr++)
    {
      ROS_INFO_STREAM("Loading " << itr->first << " path");
      joint_traj_goal.trajectory = *itr->second;
      ROS_INFO_STREAM("Filtering a joint trajectory with " << joint_traj_goal.trajectory.points.size() << " points");
      trajectory_filter_.request.trajectory = joint_traj_goal.trajectory;

      if (trajectory_filter_client.call(trajectory_filter_))
      {
        if (trajectory_filter_.response.error_code.val == trajectory_filter_.response.error_code.SUCCESS)
        {
          ROS_INFO_STREAM("======================== MOVING ROBOT ========================");
          ROS_INFO("Trajectory successfully filtered...sending goal");
          joint_traj_goal.trajectory = trajectory_filter_.response.trajectory;
          ROS_INFO_STREAM("Sending a joint trajectory with " << joint_traj_goal.trajectory.points.size() << " points");
          //joint_traj_client_ptr->sendGoal(joint_traj_goal);
          //joint_traj_client_ptr->waitForResult(ros::Duration(120));
          joint_traj_client_ptr->sendGoalAndWait(joint_traj_goal, ros::Duration(120), ros::Duration(120));
        }
      }

      else
      {
        ROS_ERROR("Failed to call filter trajectory");
      }
    }
  }

  return 0;
}
