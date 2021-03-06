
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

#include <mtconnect_cnc_robot_example/move_arm_action_clients/MovePickPlaceServer.h>

int main(int argc,char** argv)
{
	using namespace mtconnect_cnc_robot_example;

	ros::init(argc,argv,"move_pick_place_server");
	MoveArmActionClientPtr arm_client_ptr = MoveArmActionClientPtr(new MovePickPlaceServer());
	arm_client_ptr->run();

	return 0;
}
