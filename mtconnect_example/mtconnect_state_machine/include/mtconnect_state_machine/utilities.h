
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

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <mtconnect_task_parser/task.h>

namespace mtconnect_state_machine
{



bool parseTaskXml(const std::string & xml,
                  std::map<std::string, trajectory_msgs::JointTrajectoryPtr> & paths,
                  std::map<std::string, boost::shared_ptr<mtconnect::JointPoint> > & points);

bool toJointTrajectory(boost::shared_ptr<mtconnect::Path> & path,
                       trajectory_msgs::JointTrajectoryPtr & traj);
}

#endif /* UTILITIES_H_ */
