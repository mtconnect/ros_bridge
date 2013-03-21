/*
 * state_machine_definitions.h
 *
 *  Created on: Mar 15, 2013
 *      Author: ros developer 
 */

#ifndef STATE_MACHINE_DEFINITIONS_H_
#define STATE_MACHINE_DEFINITIONS_H_

#include <iostream>
#include <sstream>
#include <string.h>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
//#include <boost/msm/active_state_switching_policies.hpp> only available in boost>1.46
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <map>
#include <mtconnect_cnc_robot_example/state_machine/events.h>
#include <mtconnect_cnc_robot_example/state_machine/event_handler_interface.h>
#include <mtconnect_cnc_robot_example/state_machine/states.h>


using namespace mtconnect_cnc_robot_example::state_machine;
using namespace mtconnect_cnc_robot_example::state_machine::states;

namespace mtconnect_cnc_robot_example
{
	namespace state_machine
	{
		namespace submachines
		{

			struct SubmachineInterface
			{
				SubmachineInterface(EventHandlerInterface::Ptr p = EventHandlerInterface::Ptr(),
						std::string name = "Unimplemented",
						std::size_t indenting_level = 2):
					event_handler_ptr_(p),
					name_(name),
					indenting_level_(indenting_level),
					indenting_str_(create_indenting_string(indenting_level))

				{
					std::cout<<name_ <<" constructor"<<std::endl;
				}

				virtual void set_event_handler(EventHandlerInterface::Ptr p)
				{
					event_handler_ptr_ = p;
					std::cout<< indenting_str_ <<name_<<" submachine stored event handler\n";
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

			//-------------------  Startup Sub-machine front end ---------------------------------------//
			struct Startup_: public boost::msm::front::state_machine_def<Startup_/*,BaseState*/>, public SubmachineInterface
			{

				// initial state declaration
				typedef states::BaseState initial_state;

				// initial event
				typedef events::empty_event initial_event;

				Startup_(std::string name = "Startup"):
					SubmachineInterface(EventHandlerInterface::Ptr(),name)
				{

				}

				// on entry msm method
				template <typename Event, typename FSM>
					void on_entry(Event const &evnt , FSM &sm)
				{
					std::cout<< indenting_str_ <<"Entering "<<name_<<" sub-machine\n";
				}

				// on exit msm method
				template <typename Event, typename FSM>
					void on_exit(Event const &evnt , FSM &sm)
				{
					std::cout<< indenting_str_ <<"Leaving "<<name_<<" sub-machine\n";
				}

				struct transition_table : boost::mpl::vector<
				 //    Start          Event                     Next                   Action				         Guard
				//  +----------------+-------------------------+----------------------+-----------------------------+----------------------------+
				_row < BaseState, empty_event  , BaseState                  										>
				//  +----------------+-------------------------+----------------------+-----------------------------+----------------------------+
				>{};

		        // no-transition response.
		        template <class FSM,class Event>
				//template <class Event, class FSM>
					void no_transition(Event const& e, FSM& sm,int state)
				{
		        	states::BaseState* st =  static_cast<states::BaseState*>( sm.get_state_by_id(state));

					std::cout << "no transition from state "<< st->name_ << "( id "<<state<<")"
							<< " on event " << typeid(e).name()<< std::endl;
				}

			};
			//-------------------  Startup Sub-machine back end ---------------------------------------//
			typedef boost::msm::back::state_machine<Startup_/*,states::BaseState*/> Startup;
			typedef boost::shared_ptr<Startup> StartupPtr;

			//-------------------  Ready Sub-machine front end ---------------------------------------//
			struct Ready_: public boost::msm::front::state_machine_def<Ready_/*,BaseState*/>, public SubmachineInterface
			{
				//typedef boost::msm::active_state_switch_after_exit active_state_switch_policy; // only available in boost 1.48 or greater
				typedef boost::msm::back::state_machine<Ready_> BackendType;

				Ready_(std::string name = "Ready"):
						SubmachineInterface(EventHandlerInterface::Ptr(),name)
				{

				}

				virtual void set_event_handler(EventHandlerInterface::Ptr p)
				{
					BackendType &ref = static_cast<BackendType& >(*this);
					BaseState* state_ptr = 0;

					if(p != NULL)
					{
						std::cout<<indenting_str_ << name_<<" sub-machine passing event handler to states\n";

						state_ptr = static_cast<BaseState*>(ref.get_state<RobotReset *>());
						state_ptr->set_event_handler(p);
						state_ptr = static_cast<BaseState*>(ref.get_state<CncReset *>());
						state_ptr->set_event_handler(p);
						state_ptr = static_cast<BaseState*>(ref.get_state<PeripheralsReset*>());
						state_ptr->set_event_handler(p);

					}

					SubmachineInterface::set_event_handler(p);
				}

				// on entry msm method
				template <typename Event, typename FSM>
					void on_entry(Event const &evnt , FSM &sm)
				{
					std::cout<< indenting_str_ <<"Entering "<<name_<<" sub-machine\n";
					sm.process_event(peripherals_reset_requested());
				}

				// on exit msm method
				template <typename Event, typename FSM>
					void on_exit(Event const &evnt , FSM &sm)
				{
					std::cout<< indenting_str_ <<"Leaving "<<name_<<" sub-machine\n";
				}

		        // no-transition response.
		        template <class FSM,class Event>
					void no_transition(Event const& e, FSM& sm,int state)
				{
		        	states::BaseState* st =  static_cast<states::BaseState*>( sm.get_state_by_id(state));

					std::cout << "no transition supported from state "<< st->name_ << "( id "<<state<<")"
							<< " on event " << typeid(e).name()<< std::endl;
				}

		        // event handlers
				void handle_event(cnc_reset_completed const &evnt)
				{
					static_cast<BackendType& >(*this).process_event(robot_reset_requested());
				}

				void handle_event(peripherals_reset_completed const &evnt)
				{
					static_cast<BackendType& >(*this).process_event(cnc_reset_requested());
				}

				void handle_event(robot_reset_completed const &evnt)
				{
					// 	next event should exit this submachine
					//std::cout<<indenting_str_<<name_<<" processing event "<<typeid(all_devices_ready).name()<<std::endl;
					static_cast<BackendType& >(*this).process_event(all_devices_ready());
				}

				// initial state declaration
				typedef states::EntryState initial_state;

				// initial event
				typedef events::empty_event initial_event;

				// pseudo exit state
				struct Exit : public boost::msm::front::exit_pseudo_state<all_devices_ready>{};
				struct ExitRobotFault : public boost::msm::front::exit_pseudo_state<robot_fault_detected>{};
				struct ExitCncFault : public boost::msm::front::exit_pseudo_state<cnc_fault_detected>{};
				struct ExitGripperFault : public boost::msm::front::exit_pseudo_state<gripper_fault_detected>{};

				// transition tablesub_machines::Fault_
				typedef Ready_ p;

				// the 'transition_table' nested struct is required
				struct transition_table : boost::mpl::vector<
				 //    Start          	 Event                     		 Next                   Action				          Guard
				//  +-------------------+-------------------------------+----------------------+-----------------------------+----------------------+
				_row < EntryState    	, robot_reset_requested    		, RobotReset                    											>,
				_row < EntryState    	, cnc_reset_requested      		, CncReset                    												>,
				_row < EntryState    	, peripherals_reset_requested  	, PeripheralsReset                  										>,
				_row < EntryState    	, all_devices_ready  			, Exit				                  										>,
				//  +-------------------+-------------------------------+----------------------+-----------------------------+----------------------+
				a_row < RobotReset    	, robot_reset_completed      	, EntryState           , &p::handle_event									>,
				a_row < CncReset      	, cnc_reset_completed      		, EntryState           , &p::handle_event									>,
				a_row < PeripheralsReset, peripherals_reset_completed  	, EntryState           , &p::handle_event 									>,
				//  +-------------------+-------------------------------+----------------------+-----------------------------+----------------------+
				_row < EntryState    	, robot_fault_detected  		, ExitRobotFault	                  										>,
				_row < EntryState    	, cnc_fault_detected  			, ExitCncFault		                  										>,
				_row < EntryState    	, gripper_fault_detected  		, ExitGripperFault	                  										>
				>{};

			};

			//-------------------  Ready Sub-machine back end ---------------------------------------//
			typedef boost::msm::back::state_machine<submachines::Ready_> Ready;
			typedef boost::shared_ptr<Ready> ReadyPtr;

			//-------------------  Fault Sub-machine front end ---------------------------------------//
			struct Fault_ :  public boost::msm::front::state_machine_def<Fault_>, public SubmachineInterface
			{
				typedef boost::msm::back::state_machine<Fault_> BackendType;

				Fault_(std::string name = "Fault"):
						SubmachineInterface(EventHandlerInterface::Ptr(),name)
				{

				}

				virtual void set_event_handler(EventHandlerInterface::Ptr p)
				{
					BackendType &ref = static_cast<BackendType& >(*this);
					BaseState::Ptr robot_fault_st, cnc_fault_st, gripper_fault_st;
					BaseState* state_ptr = 0;
					if(p != NULL)
					{
						std::cout<<indenting_str_ << name_<<" sub-machine passing event handler to states\n";

				//		robot_fault_st = BaseState::Ptr(new RobotFault(p));
				//		cnc_fault_st = BaseState::Ptr(new CncFault(p));
				//		gripper_fault_st = BaseState::Ptr(new GripperFault(p));
				//
				//		ref.set_states(boost::msm::back::states_ <<
				//				static_cast<RobotFault& >(*robot_fault_st) <<
				//				static_cast<CncFault& >(*cnc_fault_st) <<
				//				static_cast<GripperFault& >(*gripper_fault_st));

						state_ptr = static_cast<BaseState*>(ref.get_state<RobotFault*>());
						state_ptr->set_event_handler(p);
						state_ptr = static_cast<BaseState*>(ref.get_state<CncFault*>());
						state_ptr->set_event_handler(p);
						state_ptr = static_cast<BaseState*>(ref.get_state<GripperFault*>());
						state_ptr->set_event_handler(p);

					}

					SubmachineInterface::set_event_handler(p);
				}

				// on entry msm method
				template <typename Event, typename FSM>
					void on_entry(Event const &evnt , FSM &sm)
				{
					std::cout<<indenting_str_ <<
							"Entering "<<name_<<" sub-machine\n";
					sm.process_event(evnt);
				}

				// on exit msm method
				template <typename Event, typename FSM>
					void on_exit(Event const &evnt , FSM &sm)
				{
					std::cout<< indenting_str_ <<
							"Leaving "<<name_<<" sub-machine\n";
				}

				// no-transition response.
				template <class FSM,class Event>
					void no_transition(Event const& e, FSM& sm,int state)
				{
					states::EntryState* st =  static_cast<states::EntryState*>( sm.get_state_by_id(state));

					std::cout << "no transition for this event "<< st->name_ << "( id "<<state<<")"
							<< " on event " << typeid(e).name()<< std::endl;
				}

				// initial state declaration
				typedef states::EntryState initial_state;

				// initial event
				typedef events::empty_event initial_event;

				// transition table
				typedef Fault_ p;

				// pseudo exit state
				struct Exit : public boost::msm::front::exit_pseudo_state<robot_fault_cleared> {};

				// the 'transition_table' nested struct is required
				struct transition_table : boost::mpl::vector<
				 //    Start          Event                     Next                   Action				         Guard
				//  +----------------+-------------------------+----------------------+-----------------------------+----------------------------+
				_row < EntryState     , empty_event    			, EntryState                   													>,
				_row < EntryState     , robot_fault_detected    , RobotFault                    												>,
				_row < EntryState     , cnc_fault_detected      , CncFault                    													>,
				_row < EntryState     , gripper_fault_detected  , GripperFault                  												>,
				//  +----------------+-------------------------+----------------------+-----------------------------+----------------------------+
				_row <RobotFault     , robot_fault_detected    , RobotFault 																	>,
				_row <RobotFault     , cnc_fault_detected      , CncFault 																		>,
				_row <RobotFault     , gripper_fault_detected  , GripperFault																	>,
				_row <RobotFault     , robot_fault_cleared     , Exit			                  												>,
				//  +----------------+-------------------------+----------------------+-----------------------------+----------------------------+
				_row < CncFault      , cnc_fault_cleared       , RobotFault 																	>,
				_row < GripperFault  , gripper_fault_cleared   , RobotFault     																>
				//  +----------------+-------------------------+----------------------+-----------------------------+----------------------------+
				>{};

			};


			//-------------------  Fault Sub-machine back end ---------------------------------------//
			typedef boost::msm::back::state_machine<Fault_> Fault;
			typedef boost::shared_ptr<Fault> FaultPtr;
		}

	}
}

#endif /* STATE_MACHINE_DEFINITIONS_H_ */
