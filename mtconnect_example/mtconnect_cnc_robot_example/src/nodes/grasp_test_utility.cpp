/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <simple_message/smpl_msg_connection.h>
#include <simple_message/simple_message.h>
#include <simple_message/socket/tcp_client.h>
#include <mtconnect_cnc_robot_example/gripper_message/gripper_message.h>
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
			ROS_ERROR("Could not read port number, usage: grasp_utility <robot ip address>");
		}
		else
		{
			ROS_INFO("Grasp action connecting to IP address: %s and port: %i", ip_address.c_str(),port_number);
			industrial::tcp_client::TcpClient robot;
			robot.init(const_cast<char*>(ip_address.c_str()), port_number);
			robot.makeConnect();

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
		ROS_ERROR("Missing command line arguments, usage: grasp_utility <robot ip address>");
	}

  return 0;
}




