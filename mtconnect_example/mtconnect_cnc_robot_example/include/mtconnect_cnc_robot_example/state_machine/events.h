/*
 * events.h
 *
 *  Created on: Mar 15, 2013
 *      Author: ros developer 
 */

#ifndef EVENTS_H_
#define EVENTS_H_

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
					in_progress_(in_progress),
					success_(success)
				{

				}

				int task_id_;
				bool in_progress_;
				bool success_;
			};

			struct fault_event
			{

			};

			// material load events
			struct material_load_requested{};
			struct material_load_completed{};
			struct material_load_failed{};
			struct material_unload_requested{};
			struct material_unload_completed{};
			struct material_unload_failed{};

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
		}
	}

}

#endif /* EVENTS_H_ */
