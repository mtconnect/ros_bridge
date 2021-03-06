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

#include <mtconnect_task_parser/task_parser.h>
#include <mtconnect_task_parser/exception.h>

#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include "boost/make_shared.hpp"
#include <ros/console.h>

namespace mtconnect
{

bool evalXmlParse(bool parse_rtn, bool required, std::string name)
{
  bool rtn = false;

  if (parse_rtn)
  {
    rtn = true;
  }
  else
  {
    if (required)
    {
      ROS_ERROR_STREAM("Failed to parse REQUIRED attribute: " << name);
      rtn = false;
    }
    else
    {
      ROS_DEBUG_STREAM("Failed to parse OPTIONAL attribute: " << name);
      rtn = true;
    }
  }
  return rtn;
}

template<typename T>
  bool attrFromXml(TiXmlElement* config, std::string name, T & member, bool required = true)
  {
    ROS_DEBUG_STREAM("XML element: " << config->Value());

    int txml_code = config->QueryValueAttribute(name, &member);
    bool txml_bool = false;
    switch (txml_code)
    {
      case TIXML_SUCCESS:
        ROS_DEBUG_STREAM(
            "Element: " << config->Value() << " successfully parsed attribute: " << name << " to " << member);
        txml_bool = true;
        break;
      case TIXML_WRONG_TYPE:
        ROS_DEBUG_STREAM("Element: " << config->Value() << " failed to parse attribute: " << name << " , wrong type");
        txml_bool = false;
        break;
      case TIXML_NO_ATTRIBUTE:
        ROS_DEBUG_STREAM(
            "Element: " << config->Value() << " failed to parse attribute: " << name << ", attribute missing");
        txml_bool = false;
        break;
      default:
        ROS_WARN_STREAM("Element: " << config->Value() << " unexpected return code from tiny xml: " << txml_code);
        txml_bool = false;
        break;
    }

    return evalXmlParse(txml_bool, required, name);

  }

template<typename T>
  bool attrFromXml(TiXmlElement* config, std::string name, std::vector<T> & list, bool required = true)
  {
    bool txml_rtn = false;

    const char *attr_list = config->Attribute(name.c_str());

    if (attr_list)
    {
      std::vector<std::string> pieces;
      boost::split(pieces, attr_list, boost::is_any_of(" "));
      for (unsigned int i = 0; i < pieces.size(); ++i)
      {
        if (pieces[i] != "")
        {
          try
          {
            ROS_DEBUG_STREAM(
                "Element: " << config->Value() << " appending item: " << pieces[i] << " to " << name << " list");
            list.push_back(boost::lexical_cast<T>(pieces[i].c_str()));
            txml_rtn = true;
          }
          catch (boost::bad_lexical_cast &e)
          {
            ROS_ERROR_STREAM(
                "Element: " << config->Value() << " attribute list: " << name << ", item: " << pieces[i] << ") could not be recast");
            txml_rtn = false;
            break;
          }
        }
      }

    }

    return evalXmlParse(txml_rtn, required, name);

  }

bool fromXml(MotionGroup & motion_group, TiXmlElement* config)
{

  ROS_DEBUG_STREAM("MotionGroup::from XML element");

  motion_group.clear();

  bool rtn = attrFromXml(config, "name", motion_group.name_)
      && attrFromXml(config, "joint_names", motion_group.joint_names_);

  if (!rtn)
  {
    ROS_ERROR("Failed to parse MotionGroup");
    motion_group.clear();
  }
  return rtn;

}

bool fromXml(JointPoint & joint_point, TiXmlElement* config,
             const std::map<std::string, boost::shared_ptr<MotionGroup> >& motion_groups)
{
  ROS_DEBUG_STREAM("JointPoint::from XML element");
  joint_point.clear();

  bool rtn = attrFromXml(config, "name", joint_point.name_, false)
      && attrFromXml(config, "joint_values", joint_point.values_)
      && attrFromXml(config, "percent_velocity", joint_point.percent_velocity_, false);

  if (rtn)
  {
    std::string group_name;
    if (attrFromXml(config, "group_name", group_name))
    {
      try
      {
        joint_point.group_ = motion_groups.at(group_name);
        unsigned int group_size = joint_point.group_->joint_names_.size();
        if (group_size != joint_point.values_.size())
        {
          ROS_ERROR_STREAM(
              "Joint group size: " << group_size << " doesn't match values size: " << joint_point.values_.size());
          rtn = false;
        }
        else
        {
          rtn = true;
        }
      }
      catch (const std::out_of_range& e)
      {
        ROS_ERROR_STREAM("Group name not valid: " << e.what());
        rtn = false;
      }
    }
    else
    {
      rtn = false;
    }
  }

  if (!rtn)
  {
    ROS_ERROR("Failed to parse JointPoint");
    joint_point.clear();
  }
  return rtn;
}

bool fromXml(JointMove & joint_move, TiXmlElement* config,
             const std::map<std::string, boost::shared_ptr<MotionGroup> >& motion_groups)
{
  bool rtn = false;

  ROS_DEBUG_STREAM("JointMove::from XML element");
  joint_move.clear();
  joint_move.point_ = boost::make_shared<JointPoint>();

  if (attrFromXml(config, "name", joint_move.name_, false))
  {
    //Just looking for first element (should we warn if there are more?)
    TiXmlElement* joint_point_xml = config->FirstChildElement("joint_point");
    if (joint_point_xml)
    {
      if (fromXml(*joint_move.point_, joint_point_xml, motion_groups))
      {
        rtn = true;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Failed to parse joint point element from JointMove");
    }
  }

  if (!rtn)
  {
    ROS_ERROR("Failed to parse JointMove");
    joint_move.clear();
  }
  return rtn;
}

bool fromXml(Path & path, TiXmlElement* config,
             const std::map<std::string, boost::shared_ptr<MotionGroup> >& motion_groups)
{
  ROS_DEBUG_STREAM("Path::from XML element");

  path.clear();

  bool rtn = false;
  if (attrFromXml(config, "name", path.name_))
  {
    for (TiXmlElement* joint_move_xml = config->FirstChildElement("joint_move"); joint_move_xml; joint_move_xml =
        joint_move_xml->NextSiblingElement("joint_move"))
    {
      JointMove move;
      move.clear();
      if (fromXml(move, joint_move_xml, motion_groups))
      {
        path.moves_.push_back(move);
        ROS_DEBUG_STREAM("Adding move to path, total size: " << path.moves_.size());
        rtn = true;
      }
      else
      {
        ROS_ERROR_STREAM("Failed to parse move element of path");
        rtn = false;
        break;
      }
    }
  }

  if (!rtn)
  {
    ROS_ERROR("Failed to parse Path");
    path.clear();
  }
  return rtn;

}

bool fromXml(Task & task, TiXmlElement* config)
{
  ROS_DEBUG_STREAM("Task::from XML element");

  task.clear();

  bool rtn = false;

  // Parsing motion groups
  for (TiXmlElement* motion_group_xml = config->FirstChildElement("motion_group"); motion_group_xml; motion_group_xml =
      motion_group_xml->NextSiblingElement("motion_group"))
  {
    MotionGroup motion_group;
    motion_group.clear();
    if (fromXml(motion_group, motion_group_xml))
    {
      task.motion_groups_[motion_group.name_] = boost::make_shared<MotionGroup>(motion_group);
      ROS_DEBUG_STREAM("Adding motion group to task, total size: " << task.motion_groups_.size());
    }
    else
    {
      //It's possible that the motion group might not be used
      //TODO: MAY WANT TO RECONSIDER WHETHER WE FAIL OR NO
      ROS_WARN_STREAM("Failed to parse motion group element of task, ignoring, will fail later if group is needed");
    }
  }

  // Parsing named points
  for (TiXmlElement* joint_point_xml = config->FirstChildElement("joint_point"); joint_point_xml; joint_point_xml =
      joint_point_xml->NextSiblingElement("joint_point"))
  {
    JointPoint joint_point;
    joint_point.clear();
    if (fromXml(joint_point, joint_point_xml, task.motion_groups_))
    {
      if (joint_point.name_.empty())
      {
        //It's possible that the joint point might not be used
        //TODO: MAY WANT TO RECONSIDER WHETHER WE FAIL OR NO
        ROS_WARN_STREAM("Failed to add joint point to task level, task level points must be named");
      }
      else
      {
        task.points_[joint_point.name_] = boost::make_shared<JointPoint>(joint_point);
        ROS_DEBUG_STREAM("Adding named joint point to task, total size: " << task.points_.size());
      }
    }
    else
    {
      //It's possible that the motion group might not be used
      //TODO: MAY WANT TO RECONSIDER WHETHER WE FAIL OR NO
      ROS_WARN_STREAM("Failed to parse motion group element of task, ignoring, will fail later if group is needed");
    }
  }

  // Parsing paths
  for (TiXmlElement* path_xml = config->FirstChildElement("path"); path_xml;
      path_xml = path_xml->NextSiblingElement("path"))
  {
    Path path;
    path.clear();
    if (fromXml(path, path_xml, task.motion_groups_))
    {
      task.paths_[path.name_] = boost::make_shared<Path>(path);
      ROS_DEBUG_STREAM("Adding path to task, total size: " << task.paths_.size());
      rtn = true;
    }
    else
    {
      //It's possible that the motion group might not be used
      ROS_WARN_STREAM("Failed to parse motion group element of task, ignoring, will fail later if group is needed");
      rtn = false;
      break;
    }
  }

  if (!rtn)
  {
    ROS_ERROR("Failed to parse Task");
    task.clear();
  }
  return rtn;

}

//TODO: This funciton need more extensive error checking
bool fromXml(Task & task, const std::string & xml)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml.c_str());
  TiXmlElement *xml_t = xml_doc.FirstChildElement("task");
  return fromXml(task, xml_t);
}

} //mtconnect
