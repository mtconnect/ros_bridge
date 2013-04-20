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

#include <ros/console.h>
#include <mtconnect_state_machine/utilities.h>
#include <mtconnect_task_parser/task_parser.h>
#include <boost/tuple/tuple.hpp>
#include "boost/make_shared.hpp"

using namespace mtconnect_state_machine;



bool mtconnect_state_machine::parseTaskXml(const std::string & xml,
                  std::map<std::string, trajectory_msgs::JointTrajectoryPtr> & paths,
                  std::map<std::string, boost::shared_ptr<mtconnect::JointPoint> > & points)
{
  typedef std::map<std::string, boost::shared_ptr<mtconnect::Path> >::iterator PathMapIter;
  bool rtn;

  mtconnect::Task task;

  if (mtconnect::fromXml(task, xml))
  {

    for (PathMapIter iter = task.paths_.begin(); iter != task.paths_.end(); ++iter)
    {
      trajectory_msgs::JointTrajectoryPtr jt(new trajectory_msgs::JointTrajectory());

      if (toJointTrajectory(iter->second, jt))
      {
        paths[iter->first] = jt;
        ROS_DEBUG_STREAM("Converted path: " << iter->first << " to joint trajectory");
        rtn = true;
      }
      else
      {
        ROS_ERROR_STREAM("Failed to convert path: " << iter->first << " to joint trajectory");
        rtn = false;
        break;
      }

    }
    ROS_DEBUG_STREAM("Converted " << task.paths_.size() << " paths to "
                    << paths.size() << " joint paths");

    ROS_DEBUG_STREAM("Copying " << task.points_.size() << " to defined points");
    points = task.points_;
  }
  else
  {
    ROS_ERROR("Failed to parse task xml string");
    rtn = false;
  }

  return rtn;

}

bool mtconnect_state_machine::toJointTrajectory(boost::shared_ptr<mtconnect::Path> & path,
                                       trajectory_msgs::JointTrajectoryPtr & traj)
{
  typedef std::vector<mtconnect::JointMove>::iterator JointMovesIter;

  // This line makes the assumption that a path is tied to a single group
  // (which is only true for out case, not in general)
  traj->joint_names = path->moves_.front().point_->group_->joint_names_;
  traj->points.clear();
  for (JointMovesIter iter = path->moves_.begin(); iter != path->moves_.end(); iter++)
  {
    ROS_DEBUG("Converting point to joint trajectory point");
    trajectory_msgs::JointTrajectoryPoint jt_point;
    jt_point.positions = iter->point_->values_;
    traj->points.push_back(jt_point);
    ROS_DEBUG_STREAM("Added point to trajectory, new size: " << traj->points.size());

  }
  return true;
}




