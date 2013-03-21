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
using namespace mtconnect_cnc_robot_example::state_machine;
using namespace mtconnect_cnc_robot_example::state_machine::submachines;

int main(int argc,char** argv)
{
//	BasicStateMachine sm;
//	sm.start();
//
//	sm.process_event(events::startup_completed()); // Should enter Ready Submachine
//
//	// Enter Fault Submachine
//	sm.process_event(events::robot_fault_detected());
//	sm.process_event(events::cnc_fault_detected());
//	sm.process_event(events::cnc_fault_cleared());
//	sm.process_event(events::robot_fault_cleared());  // should exit Fault Submachine
//
//	// Enter Ready Submachine
//	sm.process_event(events::cnc_reset_requested());
//	sm.process_event(events::cnc_reset_completed());
//	sm.process_event(events::robot_reset_completed()); // should exit Ready Submachine
//
//	// Enter Operational Submachine
//	sm.process_event(events::robot_pickup_material());
//	sm.process_event(events::robot_move_completed()); // back in RobotWaiting
//
//	sm.process_event(events::robot_approach_cnc());
//	sm.process_event(events::robot_move_completed()); // back in RobotWaiting
//
//	sm.process_event(events::robot_enter_cnc());
//	sm.process_event(events::robot_move_completed()); // back in RobotWaiting
//
//	sm.process_event(events::robot_exit_cnc());
//	sm.process_event(events::robot_move_completed()); // back in RobotWaiting
//
//	sm.process_event(events::cnc_action_requested(cnc_action_requested::CLOSE_CHUCK));
//	sm.process_event(events::cnc_action_completed(cnc_action_requested::CLOSE_CHUCK)); // back in RobotWaiting
//
//	// Enter Fault Submachine
//	sm.process_event(events::robot_fault_detected());
//	sm.process_event(events::robot_fault_cleared()); // back in RobotWaiting

	std::cout<<"Starting State Machine tests"<<std::endl;
	EventHandlerInterface::Ptr event_handler_ptr = EventHandlerInterface::Ptr(new EventHandlerInterface());

	// states instances
	BaseState::Ptr robot_fault_ptr = BaseState::Ptr(new RobotFault(event_handler_ptr));
	BaseState::Ptr cnc_fault_ptr = BaseState::Ptr(new CncFault(event_handler_ptr));
	BaseState::Ptr gripper_fault_ptr = BaseState::Ptr(new GripperFault(event_handler_ptr));

	// submachine instances
	FaultPtr f = FaultPtr(new Fault());
	ReadyPtr r = ReadyPtr(new Ready());
	StartupPtr s = StartupPtr(new Startup());

	std::cout<<"Starting Fault Submachine test"<<std::endl;

	f->set_states(boost::msm::back::states_ << static_cast<GripperFault& >(*gripper_fault_ptr) <<
			static_cast<CncFault& >(*cnc_fault_ptr) <<
			static_cast<RobotFault& >(*robot_fault_ptr));

	f->set_event_handler(event_handler_ptr);

	f->start();
	f->process_event(events::cnc_fault_detected());
	f->process_event(events::cnc_fault_cleared());

	std::cout<<"Starting Ready Submachine test"<<std::endl;

	r->set_event_handler(event_handler_ptr);

	r->start();
	r->process_event(events::cnc_reset_requested());
	r->process_event(events::cnc_reset_completed());
	r->process_event(events::peripherals_reset_completed());
	r->process_event(events::peripherals_reset_requested());
	r->process_event(events::peripherals_reset_completed());

	return 0;
}
