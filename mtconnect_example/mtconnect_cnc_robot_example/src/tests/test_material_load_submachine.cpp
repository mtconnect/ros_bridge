/*
 * test_material_load_submachine.cpp
 *
 *  Created on: Mar 19, 2013
 *      Author: ros developer 
 */

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30 //or whatever you need
#define BOOST_MPL_LIMIT_MAP_SIZE 30 //or whatever you need
#define FUSION_MAX_VECTOR_SIZE 20 // sets the desired number of states (10 is the default)
#define BOOST_MSM_CONSTRUCTOR_ARG_SIZE 5

#include <mtconnect_cnc_robot_example/state_machine/event_handler_interface.h>
#include <mtconnect_cnc_robot_example/state_machine/state_machine_definitions.h>
#include <mtconnect_cnc_robot_example/state_machine/material_handling_submachine.h>
using namespace mtconnect_cnc_robot_example::state_machine;
using namespace mtconnect_cnc_robot_example::state_machine::submachines;

int main(int argc,char** argv)
{
	EventHandlerInterface::Ptr event_handler = EventHandlerInterface::Ptr(new EventHandlerInterface());
	submachines::MaterialLoadPtr m = MaterialLoadPtr(new MaterialLoad());

	m->set_event_handler(event_handler);

	// creating task list
	std::vector<int> task_list = list_of((int)ROBOT_MOVE_HOME)
			((int)CNC_OPEN_DOOR)//1
			((int)CNC_OPEN_CHUCK)//2
			((int)ROBOT_PICKUP_MATERIAL)//3
			((int)ROBOT_APPROACH_CNC)//4
			((int)ROBOT_ENTER_CNC)//5
			((int)ROBOT_MOVE_TO_CHUCK)//6
			((int)CNC_CLOSE_CHUCK)//7
			((int)GRIPPER_OPEN)//8
			((int)ROBOT_RETREAT_FROM_CHUCK)//9
			((int)ROBOT_EXIT_CNC)//10
			((int)CNC_CLOSE_DOOR);

	std::cout<< "Starting Material Load submachine test"<<std::endl;
	m->start();

	// stepping through positive cases first
	m->process_event(events::material_load_requested());
	m->process_event(events::robot_move_completed(task_list[0])); // robot task 1 completed
	m->process_event(events::cnc_action_completed(task_list[1]));
	m->process_event(events::cnc_action_completed(task_list[2]));
	m->process_event(events::robot_move_completed(task_list[3]));
	m->process_event(events::robot_move_completed(task_list[4]));
	m->process_event(events::robot_move_completed(task_list[5]));
	m->process_event(events::robot_move_completed(task_list[6]));
	m->process_event(events::cnc_action_completed(task_list[7]));
	m->process_event(events::gripper_action_completed(task_list[8]));
	m->process_event(events::robot_move_completed(task_list[9]));
	m->process_event(events::robot_move_completed(task_list[10]));
	m->process_event(events::cnc_action_completed(task_list[11]));

	// stepping through positive cases until task # 5 and then sending task completion event out of sequence
	std::cout<< "Causing error on Material Load submachine on task 5"<<std::endl;
	m->process_event(events::material_load_requested());
	m->process_event(events::robot_move_completed(task_list[0])); // robot task 1 completed
	m->process_event(events::cnc_action_completed(task_list[1]));
	m->process_event(events::cnc_action_completed(task_list[2]));
	m->process_event(events::robot_move_completed(task_list[3]));
	m->process_event(events::robot_move_completed(task_list[4]));
	m->process_event(events::robot_move_completed(task_list[8]));

	//m->execute_queued_events();// this causes the following error
/*	terminate called after throwing an instance of
 * 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::bad_function_call> >'
	  what():  call to empty boost::function*/

	std::cout<< "Attempting to restart Material Load submachine after fault"<<std::endl;
	m->process_event(events::material_load_requested());
	m->process_event(events::robot_move_completed(task_list[0])); // robot task 1 completed
	m->process_event(events::cnc_action_completed(task_list[1]));
	m->process_event(events::cnc_action_completed(task_list[2]));
	m->process_event(events::robot_move_completed(task_list[3]));
	m->process_event(events::robot_move_completed(task_list[4]));

	return 0;
}
