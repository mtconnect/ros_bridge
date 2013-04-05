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

#include "mtconnect_task_parser/task_parser.h"

#include "boost/make_shared.hpp"

#include <gtest/gtest.h>

TEST(TaskParser, util)
{
  //TODO Should do some testing of the internal task parsing algorithms

}
TEST(MotionGroup, from_xml)
{
  using namespace std;

  const string xml_string =
      "<motion_group name=\"my_name\" joint_names=\"joint_1 joint_2 \
      joint_3 joint_4 joint_5 joint_6\"/>";

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  TiXmlElement *xml_mg = xml_doc.FirstChildElement("motion_group");
  ASSERT_TRUE(xml_mg);

  mtconnect::MotionGroup mg;
  ASSERT_TRUE(mtconnect::fromXml(mg, xml_mg));

  // Check that group name correctly id'ed
  EXPECT_EQ("my_name", mg.name_);

  // The order of joint names is important, so we test order and equivalence
  EXPECT_EQ("joint_1", mg.joint_names_[0]);
  EXPECT_EQ("joint_2", mg.joint_names_[1]);
  EXPECT_EQ("joint_3", mg.joint_names_[2]);
  EXPECT_EQ("joint_4", mg.joint_names_[3]);
  EXPECT_EQ("joint_5", mg.joint_names_[4]);
  EXPECT_EQ("joint_6", mg.joint_names_[5]);

}

TEST(JointPoint, from_xml)
{
  using namespace std;
  using namespace mtconnect;

  const string xml_string = "<joint_point joint_values=\"0 10 0 5 0 6\" group_name=\"group_1\"/>";

  boost::shared_ptr<MotionGroup> group_ptr = boost::make_shared<MotionGroup>();
  group_ptr->joint_names_.push_back("joint_1");
  group_ptr->joint_names_.push_back("joint_2");
  group_ptr->joint_names_.push_back("joint_3");
  group_ptr->joint_names_.push_back("joint_4");
  group_ptr->joint_names_.push_back("joint_5");
  group_ptr->joint_names_.push_back("joint_6");

  group_ptr->name_ = "group_1";
  map<string, boost::shared_ptr<MotionGroup> > group_map;
  group_map[group_ptr->name_] = group_ptr;

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  TiXmlElement *xml_jp = xml_doc.FirstChildElement("joint_point");
  ASSERT_TRUE(xml_jp);

  JointPoint jp;

  ASSERT_TRUE(fromXml(jp, xml_jp, group_map));

  //Parsing XML should fail with an empty group map
  group_map.clear();
  ASSERT_FALSE(fromXml(jp, xml_jp, group_map));
}

TEST(JointMove, from_xml)
{
  using namespace std;
  using namespace mtconnect;

  const string xml_string = "<joint_move> <joint_point joint_values=\"1 2 3\" group_name=\"group_1\"/></joint_move>";

  boost::shared_ptr<MotionGroup> group_ptr = boost::make_shared<MotionGroup>();
  group_ptr->joint_names_.push_back("joint_1");
  group_ptr->joint_names_.push_back("joint_2");
  group_ptr->joint_names_.push_back("joint_3");

  group_ptr->name_ = "group_1";
  map<string, boost::shared_ptr<MotionGroup> > group_map;
  group_map[group_ptr->name_] = group_ptr;

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  TiXmlElement *xml_jm = xml_doc.FirstChildElement("joint_move");
  ASSERT_TRUE(xml_jm);

  JointMove jm;

  ASSERT_TRUE(fromXml(jm, xml_jm, group_map));

}

TEST(Path, from_xml)
{
  using namespace std;
  using namespace mtconnect;

  const string xml_string = "<path name=\"path_1\">"
      "<joint_move>"
      "<joint_point joint_values=\"1 2 3\" group_name=\"group_1\"/>"
      "</joint_move>"
      "<joint_move>"
      "<joint_point joint_values=\"4 5 6\" group_name=\"group_1\"/>"
      "</joint_move>"
      "</path>";

  boost::shared_ptr<MotionGroup> group_ptr = boost::make_shared<MotionGroup>();
  group_ptr->joint_names_.push_back("joint_1");
  group_ptr->joint_names_.push_back("joint_2");
  group_ptr->joint_names_.push_back("joint_3");

  group_ptr->name_ = "group_1";
  map<string, boost::shared_ptr<MotionGroup> > group_map;
  group_map[group_ptr->name_] = group_ptr;

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  TiXmlElement *xml_p = xml_doc.FirstChildElement("path");
  ASSERT_TRUE(xml_p);

  Path p;
  ASSERT_TRUE(fromXml(p, xml_p, group_map));

  //Testing that data got initialized as expected
  ASSERT_EQ(2, p.moves_.size());
  ASSERT_EQ(3, p.moves_.front().point_->values_.size());
  //Confirm values for first point
  EXPECT_DOUBLE_EQ(1.0, p.moves_.front().point_->values_[0]);
  EXPECT_DOUBLE_EQ(2.0, p.moves_.front().point_->values_[1]);
  EXPECT_DOUBLE_EQ(3.0, p.moves_.front().point_->values_[2]);

  //Confirm values for last point
  EXPECT_DOUBLE_EQ(4.0, p.moves_.back().point_->values_[0]);
  EXPECT_DOUBLE_EQ(5.0, p.moves_.back().point_->values_[1]);
  EXPECT_DOUBLE_EQ(6.0, p.moves_.back().point_->values_[2]);

}

TEST(Task, from_xml)
{
  using namespace std;
  using namespace mtconnect;

  const string xml_string = "<task>"
      "<motion_group name=\"group_1\" joint_names=\"joint_1 joint_2 joint_3\"/>"
      "<motion_group name=\"group_2\" joint_names=\"joint_4 joint_5 joint_6\"/>"
      "<path name=\"path_1\">"
      "<joint_move>"
      "<joint_point joint_values=\"1 2 3\" group_name=\"group_1\"/>"
      "</joint_move>"
      "<joint_move>"
      "<joint_point joint_values=\"4 5 6\" group_name=\"group_2\"/>"
      "</joint_move>"
      "</path>"
      "<path name=\"path_2\">"
      "<joint_move>"
      "<joint_point joint_values=\"1 2 3\" group_name=\"group_2\"/>"
      "</joint_move>"
      "<joint_move>"
      "<joint_point joint_values=\"4 5 6\" group_name=\"group_1\"/>"
      "</joint_move>"
      "</path>"
      "</task>";

  Task task;
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  TiXmlElement *xml_t = xml_doc.FirstChildElement("task");

  ASSERT_TRUE(fromXml(task, xml_t));

  ASSERT_EQ(2, task.motion_groups_.size());
  ASSERT_EQ(2, task.paths_.size());

  boost::shared_ptr<MotionGroup> group_ptr = task.motion_groups_["group_1"];
  ASSERT_TRUE(group_ptr);

  boost::shared_ptr<Path> path_ptr = task.paths_["path_1"];
  ASSERT_EQ(path_ptr->moves_.front().point_->group_, group_ptr);

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

