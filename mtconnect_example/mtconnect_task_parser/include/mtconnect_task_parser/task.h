/*
 * Copyright 2013 Southwest Research Institute
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef MTCONNECT_TASK_H
#define MTCONNECT_TASK_H

#include <string>
#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace mtconnect
{

class MotionGroup
{
public:

  MotionGroup()
  {
    this->clear();
  }
  ;
  std::string name_;
  std::vector<std::string> joint_names_;
  void clear()
  {
    name_.clear();
    joint_names_.clear();
  }
  ;
};

class JointPoint
{
public:
  JointPoint()
  {
    this->clear();
  }
  ;
  std::string name_;
  boost::shared_ptr<MotionGroup> group_;
  std::vector<double> values_;
  double percent_velocity_;
  void clear()
  {
    name_.clear();
    group_.reset();
    values_.clear();
    percent_velocity_ = 100.0;
  }
  ;

};

class JointMove
{
public:
  JointMove()
  {
    this->clear();
  }
  ;
  std::string name_;
  boost::shared_ptr<JointPoint> point_;
  void clear()
  {
    name_.clear();
    point_.reset();
  }
  ;
};

class Path
{
public:
  Path()
  {
    this->clear();
  }
  ;
  std::string name_;
  //Default motion group may be used in the future for move/points that don't define one
  //boost::shared_ptr<MotionGroup> default_group_;
  //Joint moves should probably be pointers?
  std::vector<JointMove> moves_;
  void clear()
  {
    name_.clear();
    //default_group_.reset();
    moves_.clear();
  }
};

class Task
{
public:
  Task()
  {
    this->clear();
  }
  std::map<std::string, boost::shared_ptr<JointPoint> >points_;
  std::map<std::string, boost::shared_ptr<MotionGroup> >motion_groups_;
  std::map<std::string, boost::shared_ptr<Path> >paths_;
  void clear()
  {
    motion_groups_.clear();
    paths_.clear();
  }
};

}

#endif //TASK_H
