/*
 * state_machine_interface.h
 *
 *  Created on: Mar 27, 2013
 *      Author: ros developer 
 */

#ifndef STATE_MACHINE_INTERFACE_H_
#define STATE_MACHINE_INTERFACE_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assign/list_inserter.hpp>
#include <ros/ros.h>

namespace mtconnect_cnc_robot_example {	namespace state_machine	{

	namespace states
	{
		enum State
		{

			EXIT = int(-5),
			ROBOT_FAULT,
			CNC_FAULT,
			GRIPPER_FAULT,
			EMPTY,// = int(-1),
			DISPLAY_STATES = int(0),
			STARTUP,// = int(1),
			READY,
			ROBOT_RESET,
			CNC_RESET,
			PERIPHERALS_RESET,
			ROBOT_MOVING,
			CNC_MOVING,
			GRIPPER_MOVING,
			MATERIAL_LOAD_STARTED,
			MATERIAL_LOAD_COMPLETED,
			MATERIAL_UNLOAD_STARTED,
			MATERIAL_UNLOAD_COMPLETED,
			ROBOT_MOVE_STARTED,
			ROBOT_MOVE_COMPLETED,
		};

		static std::map<int,std::string> STATE_MAP =
				boost::assign::map_list_of(DISPLAY_STATES,"DISPLAY_STATES")
				(EMPTY,"EMPTY")
				(STARTUP,"STARTUP")
				(READY,"READY")
				(ROBOT_RESET,"ROBOT_RESET")
				(CNC_RESET,"CNC_RESET")
				(PERIPHERALS_RESET,"PERIPHERALS_RESET")
				(ROBOT_FAULT,"ROBOT_FAULT")
				(CNC_FAULT,"CNC_FAULT")
				(GRIPPER_FAULT,"GRIPPER_FAULT")
				(ROBOT_MOVING,"ROBOT_MOVING")
				(CNC_MOVING,"CNC_MOVING")
				(GRIPPER_MOVING,"GRIPPER_MOVING")
				(MATERIAL_LOAD_STARTED,"MATERIAL_LOAD_STARTED")
				(MATERIAL_LOAD_COMPLETED,"MATERIAL_LOAD_COMPLETED")
				(MATERIAL_UNLOAD_STARTED,"MATERIAL_UNLOAD_STARTED")
				(MATERIAL_UNLOAD_COMPLETED,"MATERIAL_UNLOAD_COMPLETED")
				(ROBOT_MOVE_STARTED,"ROBOT_MOVE_STARTED")
				(ROBOT_MOVE_COMPLETED,"ROBOT_MOVE_COMPLETED")
				(EXIT,"EXIT");

	}


	class StateMachineInterface
	{

	public:

		typedef boost::shared_ptr<StateMachineInterface> Ptr;

	public:
		StateMachineInterface(){}
		virtual ~StateMachineInterface(){}

		virtual void run()
		{
			ros::NodeHandle nh;
			ros::AsyncSpinner spinner(2);
			spinner.start();

			ros::Duration loop_pause(0.5f);
			set_active_state(states::STARTUP);

			int last_state = states::EMPTY;
			int active_state = get_active_state();
			print_current_state();
			while(ros::ok() && process_transition())
			{
				// getting externally entered state
				get_state_param_override();

				// printing new state info
				active_state = get_active_state();
				if(active_state != last_state)
				{
					last_state = active_state;
					print_current_state();
				}
			}
		}

		// state set and get thread safe methods
		void set_active_state(int state)
		{
			boost::mutex::scoped_lock lock(active_state_mutex_);
			previous_state_ = active_state_;
			active_state_ = state;
		}

		int get_active_state()
		{
			boost::mutex::scoped_lock lock(active_state_mutex_);
			return active_state_;
		}

		int get_previous_state()
		{
			boost::mutex::scoped_lock lock(active_state_mutex_);
			return previous_state_;
		}


		void set_fault_detected(bool fault_detected)
		{
			boost::mutex::scoped_lock lock(fault_detected_mutex_);
			fault_detected_ = fault_detected;
		}

		bool get_fault_detected()
		{
			boost::mutex::scoped_lock lock(fault_detected_mutex_);
			return fault_detected_;
		}

		// related state machine methods
		void get_state_param_override(std::string name_space = "state_override")
		{
			ros::NodeHandle nh("~");
			int state = states::EMPTY;
			if(nh.getParam(name_space,state) && state != states::EMPTY)
			{
				set_active_state(state);
				// setting back to empty state
				nh.setParam(name_space,states::EMPTY);
			}
		}


		void print_current_state()
		{
			using namespace mtconnect_cnc_robot_example::state_machine::states;
			std::cout<<"\t" <<STATE_MAP[get_active_state()]<<" state active"<<std::endl;
		}

		void print_state_list()
		{
			using namespace mtconnect_cnc_robot_example::state_machine::states;
			std::map<int,std::string>::iterator i;
			std::cout<<"\tState List\n";
			for(i = STATE_MAP.begin(); i != STATE_MAP.end(); i++)
			{
				std::cout<<"\t- ("<<i->first<<") \t"<< i->second<<"\n";
			}
		}

	protected:

		// transition actions
		virtual bool on_startup(){return true;}
		virtual bool on_ready(){return true;}
		virtual bool on_robot_reset(){return true;}
		virtual bool on_material_load_started(){return true;}
		virtual bool on_material_load_completed(){return true;}
		virtual bool on_material_unload_started(){return true;}
		virtual bool on_material_unload_completed(){return true;}
		virtual bool on_cnc_reset(){return true;}
		virtual bool on_peripherals_reset(){return true;}
		virtual bool on_robot_fault(){return true;}
		virtual bool on_cnc_fault(){return true;}
		virtual bool on_gripper_fault(){return true;}
		virtual bool on_display_states(){print_state_list(); return true;}

		// process state machine transition
		virtual bool process_transition()
		{
			switch(get_active_state())
			{
			case states::STARTUP:

				if(on_startup())
				{
					set_active_state(states::READY);
				}

				break;
			case states::READY:

				on_ready();

				break;

			case states::ROBOT_RESET:

				if(on_robot_reset())
				{
					set_active_state(states::READY);
				}

				break;

			case states::CNC_RESET:

				if(on_cnc_reset())
				{
					set_active_state(states::ROBOT_RESET);
				}

				break;

			case states::PERIPHERALS_RESET:

				if(on_peripherals_reset())
				{
					set_active_state(states::CNC_RESET);
				}

				break;

			case states::ROBOT_MOVING:

				break;

			case states::CNC_MOVING:

				break;

			case states::GRIPPER_MOVING:

				break;

			case states::ROBOT_FAULT:

				if(on_robot_fault())
				{
					set_active_state(states::READY);
				}

				break;

			case states::CNC_FAULT:

				if(on_cnc_fault())
				{
					set_active_state(states::ROBOT_FAULT);
				}
				break;

			case states::GRIPPER_FAULT:

				if(on_gripper_fault())
				{
					set_active_state(states::CNC_FAULT);
				}

				break;

			case states::MATERIAL_LOAD_STARTED:

				if(on_material_load_started())
				{
					set_active_state(states::ROBOT_MOVING);
				}

				break;
			case states::MATERIAL_LOAD_COMPLETED:

				if(on_material_load_completed())
				{
					set_active_state(states::READY);
				}

				break;

			case states::MATERIAL_UNLOAD_STARTED:

				if(on_material_unload_started())
				{
					set_active_state(states::ROBOT_MOVING);
				}
				break;
			case states::MATERIAL_UNLOAD_COMPLETED:

				if(on_material_unload_completed())
				{
					set_active_state(states::READY);
				}

				break;

			case states::DISPLAY_STATES:

				if(on_display_states())
				{
					set_active_state(get_previous_state());
				}

				break;

			case states::EXIT:

				return false;
			}

			return true;
		}

	protected:

		// event handling members
		int active_state_;
		int previous_state_;
		bool fault_detected_;

		// threading
		boost::thread action_execution_thread_;
		boost::mutex active_state_mutex_;
		boost::mutex fault_detected_mutex_;

	};

}}

#endif /* STATE_MACHINE_INTERFACE_H_ */