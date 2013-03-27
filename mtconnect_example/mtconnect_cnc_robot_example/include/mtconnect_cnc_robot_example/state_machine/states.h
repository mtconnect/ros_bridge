/*
 * states.h
 *
 *  Created on: Mar 15, 2013
 *      Author: ros developer 
 */

#ifndef STATES_H_
#define STATES_H_

#include <boost/assert.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
//#include <boost/msm/active_state_switching_policies.hpp> only available in boost>1.46
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <mtconnect_cnc_robot_example/state_machine/events.h>
#include <mtconnect_cnc_robot_example/state_machine/event_handler_interface.h>
#include <sstream>

using namespace mtconnect_cnc_robot_example::state_machine;
namespace mtconnect_cnc_robot_example
{
	namespace state_machine
	{
		namespace states
		{

			struct BaseState :public boost::msm::front::state<>
			{
			public:
				typedef boost::shared_ptr<BaseState> Ptr;

			public:
				BaseState(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),
						std::string name ="Base State",	std::size_t indenting_level = 3):
					event_handler_ptr_(p),
					name_(name),
					indenting_level_(indenting_level),
					indenting_str_(create_indenting_string(indenting_level))

				{
					std::cout<<name_ <<" constructor"<<std::endl;
				}

				~BaseState(){}

				template <typename Event, typename FSM>
					void on_entry(Event const &evnt , FSM &sm)
				{
					std::cout<< indenting_str_ <<"Entering "<<name_<<" state\n";
					if(event_handler_ptr_ != NULL)
					{
						event_handler_ptr_->handle_event(evnt);
					}
				}

				template <typename Event, typename FSM>
					void on_exit(Event const &evnt , FSM &sm)
				{
					std::cout<< indenting_str_ <<"Leaving "<<name_<<" state\n";
				}

				void set_event_handler(EventHandlerInterface::Ptr event_handler)
				{
					event_handler_ptr_ = event_handler;
					std::cout<< indenting_str_ <<name_<<" state stored event handler\n";
				}

				std::string create_indenting_string(std::size_t indenting_level)
				{
					std::stringstream ss;
					for(std::size_t i = 0; i < indenting_level; i++)
					{
						ss<<"\t";
					}

					return ss.str();
				}

				std::string name_;
				EventHandlerInterface::Ptr event_handler_ptr_;
				std::string indenting_str_;
				std::size_t indenting_level_; // used for printouts
			};


			struct EntryState :public BaseState
			{
				EntryState(std::string name = "Entry State"):
					BaseState(EventHandlerInterface::Ptr(),name)
				{

				}

				template <typename Event, typename FSM>
					void on_entry(Event const &evnt , FSM &sm)
				{
					std::cout<< indenting_str_ <<"Entering "<<name_<<" state\n";
				}

				template <typename Event, typename FSM>
					void on_exit(Event const &evnt , FSM &sm)
				{
					std::cout<< indenting_str_ <<"Leaving "<<name_<<" state\n";
				}
			};

			struct RobotWaiting : public BaseState
			{
				RobotWaiting(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Robot Waiting"):
					BaseState(EventHandlerInterface::Ptr(),name)
				{

				}
			};

			struct RobotMoving : public BaseState
			{
				RobotMoving(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Robot Moving"):
					BaseState(p,name)
				{

				}
			};

			struct CncCompletingRequest : public BaseState
			{
				CncCompletingRequest(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Cnc Completing Request"):
					BaseState(p,name)
				{

				}
			};

			struct GripperMoving : public BaseState
			{
				GripperMoving(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Gripper Moving"):
					BaseState(p,name)
				{

				}
			};

			struct RobotFault : public BaseState
			{
				RobotFault(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Robot Fault"):
					BaseState(p,name),
					robot_at_home_(false),
					robot_reset_received_(false)
				{

				}

				bool robot_at_home_;
				bool robot_reset_received_;
			};

			struct CncFault : public BaseState
			{
				CncFault(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Cnc Fault"):
					BaseState(p,name)
				{

				}
			};

			struct GripperFault : public BaseState
			{
				GripperFault(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Gripper Fault"):
					BaseState(p,name)
				{

				}
			};

			struct RobotReset : public BaseState
			{
				RobotReset(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Robot Reset"):
					BaseState(p,name)
				{

				}
			};

			struct CncReset : public BaseState
			{
				CncReset(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Cnc Reset"):
					BaseState(p,name)
				{

				}
			};

			struct PeripheralsReset : public BaseState
			{
				PeripheralsReset(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),std::string name ="Robot Peripherals Reset"):
					BaseState(p,name)
				{

				}
			};

		}

	}
}

#endif /* STATES_H_ */
