/*
 * event_handler_interface.h
 *
 *  Created on: Mar 15, 2013
 *      Author: ros developer 
 */

#ifndef EVENT_HANDLER_INTERFACE_H_
#define EVENT_HANDLER_INTERFACE_H_

#include <mtconnect_cnc_robot_example/state_machine/events.h>
#include <boost/shared_ptr.hpp>

namespace mtconnect_cnc_robot_example
{
	namespace state_machine
	{
		using namespace mtconnect_cnc_robot_example::state_machine::events;
		class EventHandlerInterface
		{
		public:

			typedef boost::shared_ptr<EventHandlerInterface> Ptr;

		public:

			EventHandlerInterface()
			{

			}

			virtual ~EventHandlerInterface()
			{

			}

			// event handling virtual methods

			virtual bool handle_event(const task_event &evnt) {return true;}
			virtual bool handle_event(const fault_event &evnt){return true;}
			virtual bool handle_event(const empty_event &evnt){return true;}

			virtual bool handle_event(const material_load_requested &evnt){return true;}
			virtual bool handle_event(const material_load_completed &evnt){return true;}
			virtual bool handle_event(const material_load_failed &evnt){return true;}
			virtual bool handle_event(const material_unload_requested &evnt){return true;}
			virtual bool handle_event(const material_unload_completed &evnt){return true;}
			virtual bool handle_event(const material_unload_failed &evnt){return true;}

			virtual bool handle_event(const robot_move_requested &evnt){return true;}
			virtual bool handle_event(const robot_move_completed &evnt){return true;}
			virtual bool handle_event(const robot_move_update &evnt){return true;}
			virtual bool handle_event(const robot_fault_detected &evnt){return true;}
			virtual bool handle_event(const robot_fault_cleared &evnt){return true;}
			virtual bool handle_event(const robot_reset_requested &evnt){return true;}
			virtual bool handle_event(const robot_reset_completed &evnt){return true;}

			virtual bool handle_event(const cnc_action_requested &evnt){return true;}
			virtual bool handle_event(const cnc_action_completed &evnt){return true;}
			virtual bool handle_event(const cnc_action_update &evnt){return true;}
			virtual bool handle_event(const cnc_fault_detected &evnt){return true;}
			virtual bool handle_event(const cnc_fault_cleared &evnt){return true;}
			virtual bool handle_event(const cnc_reset_requested &evnt){return true;}
			virtual bool handle_event(const cnc_reset_completed &evnt){return true;}

			virtual bool handle_event(const gripper_action_requested &evnt){return true;}
			virtual bool handle_event(const gripper_action_completed &evnt){return true;}
			virtual bool handle_event(const gripper_action_update &evnt){return true;}
			virtual bool handle_event(const gripper_fault_detected &evnt){return true;}
			virtual bool handle_event(const gripper_fault_cleared &evnt){return true;}

			virtual bool handle_event(const peripherals_reset_requested &evnt){return true;}
			virtual bool handle_event(const peripherals_reset_completed &evnt){return true;}
			virtual bool handle_event(const startup_completed &evnt){return true;}
		};
	}

}


#endif /* EVENT_HANDLER_INTERFACE_H_ */
