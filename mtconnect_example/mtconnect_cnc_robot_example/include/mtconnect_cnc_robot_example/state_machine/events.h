/*
 * events.h
 *
 *  Created on: Mar 15, 2013
 *      Author: ros developer 
 */

#ifndef EVENTS_H_
#define EVENTS_H_

#include <string>

namespace mtconnect_cnc_robot_example
{
	namespace state_machine
	{

		namespace events
		{
			struct empty_event
			{

			};

			struct task_event
			{
				task_event(int task_id = 0, bool in_progress = true,bool success = true):
					task_id_(task_id),
					task_description_(""),
					in_progress_(in_progress),
					success_(success)
				{

				}

				int task_id_;
				std::string task_description_;
				bool in_progress_;
				bool success_;
			};

			struct fault_event
			{
				fault_event(int task_id = 0, int error_code = 0,
						std::string task_desc = "", std::string error_desc = ""):
					task_id_(task_id),
					task_description_(task_desc),
					error_code_(error_code),
					error_description_(error_desc)
				{

				}

				int task_id_;
				std::string task_description_;
				int error_code_;
				std::string error_description_;
			};

			// material load events
			struct material_load_requested{};
			struct material_load_completed{};
			struct material_load_failed : public fault_event
			{
				material_load_failed(int error_code = 0 , std::string error_desc = ""):
					fault_event(0,error_code,"",error_desc)
				{

				}

				// conversion constructor (needed for event forwarding during sub-machine exit operations)
				material_load_failed(material_load_failed const &evnt):
					fault_event(evnt.task_id_,evnt.error_code_,evnt.task_description_,evnt.error_description_)
				{

				}
			};

			struct material_unload_requested{};
			struct material_unload_completed{};
			struct material_unload_failed : public fault_event
			{
				material_unload_failed(int error_code = 0 , std::string error_desc = ""):
					fault_event(0,error_code,"",error_desc)
				{

				}

				// conversion constructor (needed for event forwarding during sub-machine exit operations)
				material_unload_failed(material_unload_failed const &evnt):
					fault_event(evnt.task_id_,evnt.error_code_,evnt.task_description_,evnt.error_description_)
				{

				}

			};

			// robot operational events
			struct robot_move_requested: public task_event
			{
				robot_move_requested(int task_id = 0):
					task_event(task_id)
				{

				}
			};
			struct robot_move_completed: public task_event
			{
				robot_move_completed(int task_id = 0):
					task_event(task_id)
				{

				}
			};
			struct robot_move_update: public task_event
			{
				robot_move_update(int task_id = 0):
					task_event(task_id)
				{

				}
			};

			// robot fault events
			struct robot_fault_detected : public fault_event {};
			struct robot_fault_cleared : public fault_event {};

			// robot ready events
			struct robot_reset_requested{};
			struct robot_reset_completed{};

			// cnc operational events
			struct cnc_action_requested: public task_event
			{
				cnc_action_requested(int task_id = 0):
					task_event(task_id)
				{

				}

			};
			struct cnc_action_completed : public task_event
			{
				cnc_action_completed(int task_id = 0):
					task_event(task_id)
				{

				}
			};
			struct cnc_action_update : public task_event
			{
				cnc_action_update(int task_id = 0):
					task_event(task_id)
				{

				}
			};

			// cnc fault events
			struct cnc_fault_detected : public fault_event {};
			struct cnc_fault_cleared : public fault_event {};

			// cnc ready events
			struct cnc_reset_requested{};
			struct cnc_reset_completed{};

			// gripper operation events
			struct gripper_action_requested: public task_event
			{
				gripper_action_requested(int task_id = 0):
					task_event(task_id)
					//open_gripper_(true)
				{

				}

				//bool open_gripper_;
			};
			struct gripper_action_completed: public gripper_action_requested
			{
				gripper_action_completed(int task_id = 0):
					gripper_action_requested(task_id)
				{

				}
			};
			struct gripper_action_update: public gripper_action_requested
			{
				gripper_action_update(int task_id = 0):
					gripper_action_requested(task_id)
				{

				}
			};

			// gripper fault events
			struct gripper_fault_detected : public fault_event {};
			struct gripper_fault_cleared : public fault_event {};

			// gripper ready events
			struct peripherals_reset_requested{};
			struct peripherals_reset_completed{};

			// other events
			struct startup_completed{};
			struct all_devices_ready{};
		}
	}

}

#endif /* EVENTS_H_ */
