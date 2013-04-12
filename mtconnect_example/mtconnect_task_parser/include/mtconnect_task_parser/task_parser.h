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

#ifndef MTCONNECT_TASK_PARSER_H
#define MTCONNECT_TASK_PARSER_H


#include <tinyxml.h>
#include <mtconnect_task_parser/task.h>

namespace mtconnect
{

bool fromXml(Task & task, const std::string & xml);
bool fromXml(MotionGroup & motion_group, TiXmlElement* config);
bool fromXml(JointPoint & joint_point, TiXmlElement* config,
             const std::map<std::string, boost::shared_ptr<MotionGroup> >& motion_groups);
bool fromXml(JointMove & joint_move, TiXmlElement* config,
             const std::map<std::string, boost::shared_ptr<MotionGroup> >& motion_groups);
bool fromXml(Path & path, TiXmlElement* config,
             const std::map<std::string, boost::shared_ptr<MotionGroup> >& motion_groups);
bool fromXml(Task & task, TiXmlElement* config);


} //mtconnect

#endif //TASK_PARSER_H
