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
#include <actionlib/server/action_server.h>

#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionGoal.h>
#include <simple_message/socket/tcp_client.h>
#include <simple_message/smpl_msg_connection.h>
#include <simple_message/simple_message.h>
#include <mtconnect_grasp_action/gripper_message.h>

using namespace object_manipulation_msgs;
using namespace actionlib;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace mtconnect_cnc_robot_example::gripper_message;

static const std::string PARAM_IP_ADDRESS = "ip_address";
static const std::string PARAM_PORT_NUMBER = "port_number";

class GraspExecutionAction
{
private:
  typedef ActionServer<GraspHandPostureExecutionAction> GEAS;
  typedef GEAS::GoalHandle GoalHandle;
public:
  GraspExecutionAction(ros::NodeHandle &n, SmplMsgConnection *robot) :
    node_(n),
    action_server_(node_, "grasp_execution_action",
                   boost::bind(&GraspExecutionAction::goalCB, this, _1),
                   boost::bind(&GraspExecutionAction::cancelCB, this, _1),
                   false)
  {
    ros::NodeHandle pn("~");

    robot_ = robot;
    robot_->makeConnect();
    action_server_.start();
    
    ROS_INFO("Grasp execution action node started");

    /*
    //A little bit of debug
    GripperMessage gMsg;
    SimpleMessage request;
    SimpleMessage reply;

    gMsg.init(GripperOperationTypes::INIT);
    gMsg.toRequest(request);
    this->robot_->sendAndReceiveMsg(request, reply);
    ROS_INFO("Gripper initialized");

    gMsg.init(GripperOperationTypes::CLOSE);
    gMsg.toRequest(request);
    this->robot_->sendAndReceiveMsg(request, reply);
    ROS_INFO("Gripper closed");

    gMsg.init(GripperOperationTypes::OPEN);
    gMsg.toRequest(request);
    this->robot_->sendAndReceiveMsg(request, reply);
    ROS_INFO("Gripper opened");
    */
  }

  ~GraspExecutionAction()
  {
  }

private:


  void goalCB(GoalHandle gh)
  {
    GripperMessage gMsg;
    SimpleMessage request;
    SimpleMessage reply;

    ROS_DEBUG("Received grasping goal");

    while (!(robot_->isConnected()))
    {
      ROS_DEBUG("Reconnecting");
      robot_->makeConnect();
    }
      

    switch(gh.getGoal()->goal)
    {
      case GraspHandPostureExecutionGoal::PRE_GRASP:

        gh.setAccepted();
        ROS_WARN("Pre-grasp is not supported by this gripper");
        gh.setSucceeded();
        break;

      case GraspHandPostureExecutionGoal::GRASP:
      case GraspHandPostureExecutionGoal::RELEASE:

        gh.setAccepted();
        switch(gh.getGoal()->goal)
        {
          case GraspHandPostureExecutionGoal::GRASP:
            ROS_INFO("Executing a gripper grasp");
            gMsg.init(GripperOperationTypes::CLOSE);
            break;
          case GraspHandPostureExecutionGoal::RELEASE:
            ROS_INFO("Executing a gripper release");
            gMsg.init(GripperOperationTypes::OPEN);
            break;
        }
        gMsg.toRequest(request);
        this->robot_->sendAndReceiveMsg(request, reply);

        switch(reply.getReplyCode())
        {
          case ReplyTypes::SUCCESS:
            ROS_INFO("Robot gripper returned success");
            gh.setSucceeded();
            break;
          case ReplyTypes::FAILURE:
            ROS_ERROR("Robot gripper returned failure");
            gh.setCanceled();
            break;
        }
        break;

          default:
            gh.setRejected();
            break;

    }
  }

  void cancelCB(GoalHandle gh)
  {
    ROS_ERROR("Action cancel not supported for grasp execution action");
    gh.setRejected();
  }


  ros::NodeHandle node_;
  GEAS action_server_;

  industrial::smpl_msg_connection::SmplMsgConnection *robot_;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grasp_execution_action_node");
	ros::NodeHandle nh("~");
	std::string ip_address;
	int port_number;

	// reading parameters and proceeding
	if(nh.getParam(PARAM_IP_ADDRESS,ip_address) && nh.getParam(PARAM_PORT_NUMBER,port_number))
	{
		ROS_INFO("Grasp action connecting to IP address: %s and port: %i", ip_address.c_str(),port_number);
		industrial::tcp_client::TcpClient robot;

		robot.init(const_cast<char*>(ip_address.c_str()), port_number);
		GraspExecutionAction ge(nh, &robot);

		ros::spin();
	}
	else
	{
		ROS_ERROR("Missing 'ip_address' and 'port_number' private parameters");
	}


  return 0;
}




