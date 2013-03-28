/*
 * test_state_machine.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: ros developer 
 */

#include <mtconnect_cnc_robot_example/state_machine/state_machine_interface.h>
using namespace mtconnect_cnc_robot_example::state_machine;

#define SET_AND_WAIT_UNTIL_ACTIVE(sm,st) sm->set_active_state(st);while(sm->get_active_state() != st){};ros::Duration(0.5f).sleep()

int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_state_machine");
	ros::NodeHandle nh;

	StateMachineInterface::Ptr s(new StateMachineInterface());

	boost::thread state_machine_thread = boost::thread(&StateMachineInterface::run,s.get());

	//s->run();
	ros::Duration(0.5f).sleep();
	while(s->get_active_state()!= states::READY){};

	std::cout<<"Setting "<<states::STATE_MAP[states::MATERIAL_LOAD_STARTED]<<" active"<<std::endl;
	//s->set_active_state(states::MATERIAL_LOAD_STARTED);
	SET_AND_WAIT_UNTIL_ACTIVE(s,states::MATERIAL_LOAD_STARTED);

	std::cout<<"Setting "<<states::STATE_MAP[states::ROBOT_MOVING]<<" active"<<std::endl;
	//s->set_active_state(states::ROBOT_MOVING);
	SET_AND_WAIT_UNTIL_ACTIVE(s,states::ROBOT_MOVING);



	std::cout<<"Setting "<<states::STATE_MAP[states::CNC_MOVING]<<" active"<<std::endl;
	SET_AND_WAIT_UNTIL_ACTIVE(s,states::CNC_MOVING);
	//s->set_active_state(states::CNC_MOVING);

	std::cout<<"Setting "<<states::STATE_MAP[states::MATERIAL_LOAD_COMPLETED]<<" active"<<std::endl;
	SET_AND_WAIT_UNTIL_ACTIVE(s,states::MATERIAL_LOAD_COMPLETED);
	//s->set_active_state(states::MATERIAL_LOAD_COMPLETED);

	std::cout<<"Setting "<<states::STATE_MAP[states::CNC_FAULT]<<" active"<<std::endl;
	SET_AND_WAIT_UNTIL_ACTIVE(s,states::CNC_FAULT);

	std::cout<<"Setting "<<states::STATE_MAP[states::ROBOT_FAULT]<<" active"<<std::endl;
	SET_AND_WAIT_UNTIL_ACTIVE(s,states::ROBOT_FAULT);

	std::cout<<"Setting "<<states::STATE_MAP[states::READY]<<" active"<<std::endl;
	SET_AND_WAIT_UNTIL_ACTIVE(s,states::READY);

	if(s->get_active_state() == states::READY)
	{
		std::cout<<states::STATE_MAP[states::READY]<< " state validation passed"<<std::endl;
	}

	s->set_active_state(states::EXIT);
	state_machine_thread.join();

	return 0;
}

