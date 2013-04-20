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


#include <simple_message/smpl_msg_connection.h>
#include <simple_message/simple_message.h>
#include <simple_message/socket/tcp_client.h>
#include <mtconnect_grasp_action/gripper_message.h>
#include <iostream>
#include <sstream>
using namespace std;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace mtconnect_cnc_robot_example::gripper_message;


int main(int argc, char** argv)
{
	std::string ip_address;
	int port_number;
	std::stringstream ss;
	
	if(argc == 3)  // expects ip_address and port_number
	{

		// parsing arguments
		ip_address = std::string(argv[1]);
		ss<<argv[2];
		if(!(ss >> port_number))
		{
			ROS_ERROR("Could not read port number, usage: grasp_utility <robot ip address> <port number>");
		}
		else
		{
			ROS_INFO("Grasp action connecting to IP address: %s and port: %i", ip_address.c_str(),port_number);
			industrial::tcp_client::TcpClient robot;
			robot.init(const_cast<char*>(ip_address.c_str()), port_number);
			if(!robot.makeConnect())
			{
				ROS_ERROR_STREAM("Failed to connect, exiting");
				return 0;
			}

			cout << "Grasp client connected" << endl;;

			int i = 0;
			while (true)
			{
			  cout << "Grasp Utility" << endl
				 << "1. INIT" << endl
				 << "2. CLOSE" << endl
				 << "3. OPEN" << endl;
			  cin >> i;

			  GripperMessage gMsg;
			  SimpleMessage request;
			  SimpleMessage reply;

			  switch(i)
			  {
			  case 1:
				  gMsg.init(GripperOperationTypes::INIT);
				  gMsg.toRequest(request);
				  robot.sendAndReceiveMsg(request, reply);
				  cout << "Gripper initialized" << endl;
			    break;

			  case 2:
			    gMsg.init(GripperOperationTypes::CLOSE);
			    gMsg.toRequest(request);
			    cout << "Message length: " << request.getMsgLength() << endl;
			    robot.sendAndReceiveMsg(request, reply);
			    cout << "Gripper closed" << endl;
			    break;

			  case 3:
			    gMsg.init(GripperOperationTypes::OPEN);
			    gMsg.toRequest(request);
			    robot.sendAndReceiveMsg(request, reply);
			    cout << "Gripper opened" << endl;
			    break;

			  default:
			    return 0;
			  }
			}
		}
	}
	else
	{
		ROS_ERROR("Missing command line arguments, usage: grasp_utility <robot ip address>< port number>");
	}

  return 0;
}




