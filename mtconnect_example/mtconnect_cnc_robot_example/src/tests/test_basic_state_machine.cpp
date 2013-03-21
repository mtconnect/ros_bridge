/*
 * test_basic_state_machine.cpp
 *
 *  Created on: Mar 12, 2013
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
#include <mtconnect_cnc_robot_example/state_machine/basic_state_machine.h>
using namespace mtconnect_cnc_robot_example::state_machine;
using namespace mtconnect_cnc_robot_example::state_machine::submachines;

int main(int argc,char** argv)
{
	EventHandlerInterface::Ptr event_handler_ptr = EventHandlerInterface::Ptr(new EventHandlerInterface());
	BasicStateMachinePtr b = BasicStateMachinePtr(new BasicStateMachine());
	b->set_event_handler(event_handler_ptr);

	// creating material load task list
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

	std::cout<<"Starting Basic State Machine test"<<std::endl;

	b->start();
	b->process_event(events::startup_completed()); // should go into Ready sub-machine PeripheralsReset state

	std::cout<<"Transition from Startup to Ready sub-machine"<<std::endl;

	// Ready sub-machine
	b->process_event(events::peripherals_reset_completed()); // should go into CncReset State
	b->process_event(events::cnc_reset_completed()); // should go into RobotReset State
	b->process_event(events::robot_reset_completed()); // should exit out of Ready sub machine and into Operational sub-machine

	std::cout<<"Transition from Ready to Operational sub-machine"<<std::endl;

	//Operational sub-machine (positive cases)
	b->process_event(events::material_load_requested()); // should go into MaterialLoad sub-machine

	std::cout<<"Internal transition from Operational to Operational->MaterialLoad sub-machine"<<std::endl;

	// Operational -> MaterialLoad sub-machine
	b->process_event(events::robot_move_completed(task_list[0])); // robot task 1 completed
	b->process_event(events::cnc_action_completed(task_list[1]));
	b->process_event(events::cnc_action_completed(task_list[2]));
	b->process_event(events::robot_move_completed(task_list[3]));
	b->process_event(events::robot_move_completed(task_list[4]));
	b->process_event(events::robot_move_completed(task_list[5]));
	b->process_event(events::robot_move_completed(task_list[6]));
	b->process_event(events::cnc_action_completed(task_list[7]));
	b->process_event(events::gripper_action_completed(task_list[8]));
	b->process_event(events::robot_move_completed(task_list[9]));
	b->process_event(events::robot_move_completed(task_list[10]));
	b->process_event(events::cnc_action_completed(task_list[11]));// should exit MaterialLoad sub-machine
																  // and into Operational->RobotWaiting

	std::cout<<"External transition from Operational->MaterialLoad to Operational sub-machine"<<std::endl;

	//Operational sub-machine
	b->process_event(events::material_load_requested()); // should go into MaterialLoad sub-machine

	// Operational -> MaterialLoad sub-machine (will cause fault on task 5)
	b->process_event(events::robot_move_completed(task_list[0])); // robot task 1 completed
	b->process_event(events::cnc_action_completed(task_list[1]));
	b->process_event(events::cnc_action_completed(task_list[2]));
	b->process_event(events::robot_move_completed(task_list[3]));
	b->process_event(events::robot_move_completed(task_list[4]));
	b->process_event(events::robot_move_completed(task_list[8])); // shoudl exit out of MaterialLoad sub-machine
																  // and into Operation sub-machine


	// Operational (should forward robot, cnc or gripper fault event and enter Fault sub-machine

	std::cout<<"Transition from Operational to Fault sub-machine"<<std::endl;

	//Fault sub-machine
	b->process_event(events::cnc_fault_detected()); // forcing cnc fault
	b->process_event(events::cnc_fault_cleared()); // should go back to RobotFault
	b->process_event(events::robot_fault_cleared()); // should exit out of Fault sub-machine
													 // and enter Ready sub-machine

	std::cout<<"Transition from Fault to Ready sub-machine"<<std::endl;

	// Ready sub-machine
	b->process_event(events::peripherals_reset_completed()); // should go into CncReset State
	b->process_event(events::cnc_reset_completed()); // should go into RobotReset State
	b->process_event(events::cnc_fault_detected()); // should exit out of Ready sub machine and enter Fault sub-machine

	std::cout<<"Transition from Ready to Fault sub-machine"<<std::endl;

	//Fault sub-machine
	b->process_event(events::cnc_fault_cleared()); // should go back to RobotFault
	b->process_event(events::gripper_fault_detected()); // should go into GripperFault
	b->process_event(events::gripper_fault_cleared()); // should go back to RobotFault
	b->process_event(events::robot_fault_cleared()); // should exit out of Fault sub-machine
													 // and enter Ready sub-machine

	std::cout<<"Transition from Fault to Ready sub-machine"<<std::endl;

	// Ready sub-machine (completing all reset operations and go to Operational submachine
	b->process_event(events::peripherals_reset_completed()); // should go into CncReset State
	b->process_event(events::cnc_reset_completed()); // should go into RobotReset State
	b->process_event(events::robot_reset_completed()); // should exit out of Ready sub machine and into Operational sub-machine

	std::cout<<"External transition from Ready to Operational sub-machine"<<std::endl;






	return 0;
}
