/*
 * material_handling_submachine.h
 *
 *  Created on: Mar 15, 2013
 *      Author: ros developer 
 */

#ifndef MATERIAL_HANDLING_SUBMACHINE_H_
#define MATERIAL_HANDLING_SUBMACHINE_H_

#include <mtconnect_cnc_robot_example/state_machine/state_machine_definitions.h>
#include <boost/assign/list_inserter.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assign.hpp>

using namespace mtconnect_cnc_robot_example::state_machine;
using namespace mtconnect_cnc_robot_example::state_machine::submachines;
using namespace boost::assign;
namespace mtconnect_cnc_robot_example
{
	namespace state_machine
	{
//		namespace states
//		{
//			struct MaterialTaskDispatch : public BaseState
//			{
//				MaterialTaskDispatch(std::vector<int> task_list,std::string name ="Material Task Dispatch"):
//					BaseState(EventHandlerInterface::Ptr(),name),
//					task_list_(task_list),
//					current_task_index_(0),
//					ready_for_request_(false)
//				{
//
//				}
//
//				template <typename Event, typename FSM>
//					void on_entry(Event const &evnt , FSM &sm)
//				{
//					std::cout<< indenting_str_ <<"Entering "<<name_<<" state\n";
//					if(event_handler_ptr_ != NULL)
//					{
//						event_handler_ptr_->handle_event(evnt);
//					}
//				}
//
//				template <typename Event, typename FSM>
//					void on_exit(Event const &evnt , FSM &sm)
//				{
//					std::cout<< indenting_str_ <<"Leaving "<<name_<<" state\n";
//				}
//
//				void dispatch_task(int task_id)
//				{
//					// reference to back-end sm derived implementation
//					BackendType &ref = static_cast<BackendType&>(*this);
//
//					// events objects
//					switch(task_id)
//					{
//					case ROBOT_APPROACH_CNC:
//					case ROBOT_ENTER_CNC:
//					case ROBOT_EXIT_CNC:
//					case ROBOT_MOVE_HOME:
//					case ROBOT_MOVE_TO_CHUCK:
//					case ROBOT_PICKUP_MATERIAL:
//					case ROBOT_PLACE_WORKPIECE:
//					case ROBOT_RETREAT_FROM_CHUCK:
//					case ROBOT_CARTESIAN_MOVE:
//
//						robot_move_req_.task_id_ = task_id;
//						ref.process_event(robot_move_req_);
//						break;
//
//					case CNC_CLOSE_CHUCK:
//					case CNC_OPEN_CHUCK:
//					case CNC_CLOSE_DOOR:
//					case CNC_OPEN_DOOR:
//
//						cnc_action_req_.task_id_ = task_id;
//						ref.process_event(cnc_action_req_);
//						break;
//
//					case GRIPPER_CLOSE:
//					case GRIPPER_OPEN:
//
//						gripper_action_req_.task_id_ = task_id;
//						ref.process_event(gripper_action_req_);
//						break;
//					}
//				}
//
//				std::vector<int > task_list_;
//				int current_task_index_;
//				bool ready_for_request_;
//
//				// frequently used events
//				robot_move_requested robot_move_req_;
//				cnc_action_requested cnc_action_req_;
//				gripper_action_requested gripper_action_req_;
//			};
//
//		}

		namespace submachines
		{
			enum MaterialTask
			{
				NO_TASK = int(0),
				ROBOT_PLACE_WORKPIECE,
				ROBOT_PICKUP_MATERIAL,
				ROBOT_APPROACH_CNC,
				ROBOT_ENTER_CNC,
				ROBOT_MOVE_TO_CHUCK,
				ROBOT_RETREAT_FROM_CHUCK,
				ROBOT_EXIT_CNC,
				ROBOT_MOVE_HOME,
				ROBOT_CARTESIAN_MOVE,
				CNC_OPEN_DOOR,
				CNC_CLOSE_DOOR,
				CNC_OPEN_CHUCK,
				CNC_CLOSE_CHUCK,
				GRIPPER_OPEN,
				GRIPPER_CLOSE,
			};

			enum ErrorType
			{
				OK = int(0),
				ROBOT_ERROR,
				CNC_ERROR,
				GRIPPER_ERROR,
				BUSY_ERROR,
				UNKNOWN_ERROR
			};

			static std::map<int,std::string> MaterialTaskDescriptionMap =
					boost::assign::map_list_of(NO_TASK,"NO TASK")
			(ROBOT_PLACE_WORKPIECE,"ROBOT_PLACE_WORKPIECE")
			(ROBOT_PICKUP_MATERIAL,"ROBOT_PICKUP_MATERIAL")
			(ROBOT_APPROACH_CNC,"ROBOT_APPROACH_CNC")
			(ROBOT_ENTER_CNC,"ROBOT_ENTER_CNC")
			(ROBOT_MOVE_TO_CHUCK,"ROBOT_MOVE_TO_CHUCK")
			(ROBOT_RETREAT_FROM_CHUCK,"ROBOT_RETREAT_FROM_CHUCK")
			(ROBOT_EXIT_CNC,"ROBOT_EXIT_CNC")
			(ROBOT_MOVE_HOME,"ROBOT_MOVE_HOME")
			(ROBOT_CARTESIAN_MOVE,"ROBOT_CARTESIAN_MOVE")
			(CNC_OPEN_DOOR,"CNC_OPEN_DOOR")
			(CNC_CLOSE_DOOR,"CNC_CLOSE_DOOR")
			(CNC_OPEN_CHUCK,"CNC_OPEN_CHUCK")
			(CNC_CLOSE_CHUCK,"CNC_CLOSE_CHUCK")
			(GRIPPER_OPEN,"GRIPPER_OPEN")
			(GRIPPER_CLOSE,"GRIPPER_CLOSE");

			static std::map<int,std::string> ErrorTypeDescriptionMap =
					boost::assign::map_list_of(OK,"OK")
			(ROBOT_ERROR,"ROBOT_ERROR")
			(CNC_ERROR,"CNC_ERROR")
			(GRIPPER_ERROR,"GRIPPER_ERROR")
			(BUSY_ERROR,"BUSY_ERROR")
			(UNKNOWN_ERROR,"UNKNOWN_ERROR");

			//-------------------  Material Handling Sub-machine front end ---------------------------------------//
			struct MaterialHandling_: public boost::msm::front::state_machine_def<MaterialHandling_>, public SubmachineInterface
			{
				typedef boost::msm::back::state_machine<MaterialHandling_> BackendType;

				// initial state declaration
				typedef states::BaseState initial_state;

				// initial event
				typedef events::empty_event initial_event;

				MaterialHandling_(std::string name = "Material Handling"):
					SubmachineInterface(EventHandlerInterface::Ptr(),name),
					task_list_(),
					current_task_index_(0),
					ready_for_request_(true)
				{
					using namespace states;
					using namespace boost::assign;

					// creating task sequence
					task_list_.clear();
					task_list_ = list_of((int)ROBOT_MOVE_HOME)
							((int)CNC_OPEN_DOOR)
							((int)CNC_OPEN_CHUCK)
							((int)ROBOT_PICKUP_MATERIAL)
							((int)ROBOT_APPROACH_CNC)
							((int)ROBOT_ENTER_CNC)
							((int)ROBOT_MOVE_TO_CHUCK)
							((int)CNC_CLOSE_CHUCK)
							((int)GRIPPER_OPEN)
							((int)ROBOT_RETREAT_FROM_CHUCK)
							((int)ROBOT_EXIT_CNC)
							((int)CNC_CLOSE_DOOR);
				}

				// material load/unload members
				std::vector<int > task_list_;
				int current_task_index_;
				bool ready_for_request_;

				// frequently used events
				robot_move_requested robot_move_req_;
				cnc_action_requested cnc_action_req_;
				gripper_action_requested gripper_action_req_;

				void dispatch_task(int task_id)
				{
					// reference to back-end sm derived implementation
					BackendType &ref = static_cast<BackendType&>(*this);

					// events objects
					switch(task_id)
					{
					case ROBOT_APPROACH_CNC:
					case ROBOT_ENTER_CNC:
					case ROBOT_EXIT_CNC:
					case ROBOT_MOVE_HOME:
					case ROBOT_MOVE_TO_CHUCK:
					case ROBOT_PICKUP_MATERIAL:
					case ROBOT_PLACE_WORKPIECE:
					case ROBOT_RETREAT_FROM_CHUCK:
					case ROBOT_CARTESIAN_MOVE:

						robot_move_req_.task_id_ = task_id;
						robot_move_req_.task_description_ = MaterialTaskDescriptionMap[task_id];
						ref.process_event(robot_move_req_);
						break;

					case CNC_CLOSE_CHUCK:
					case CNC_OPEN_CHUCK:
					case CNC_CLOSE_DOOR:
					case CNC_OPEN_DOOR:

						cnc_action_req_.task_id_ = task_id;
						cnc_action_req_.task_description_ = MaterialTaskDescriptionMap[task_id];
						ref.process_event(cnc_action_req_);
						break;

					case GRIPPER_CLOSE:
					case GRIPPER_OPEN:

						gripper_action_req_.task_id_ = task_id;
						gripper_action_req_.task_description_ = MaterialTaskDescriptionMap[task_id];
						ref.process_event(gripper_action_req_);
						break;
					}
				}

				virtual bool verify_completion_and_proceed(int task_id)
				{
					return true;
				}

				virtual void set_event_handler(EventHandlerInterface::Ptr p)
				{
					BackendType &ref = static_cast<BackendType& >(*this);
					BaseState* state_ptr = 0;
					if(p != NULL)
					{
						std::cout<<indenting_str_ << name_<<" sub-machine passing event handler to states\n";

						state_ptr = static_cast<BaseState*>(ref.get_state<RobotMoving*>());
						state_ptr->set_event_handler(p);
						state_ptr = static_cast<BaseState*>(ref.get_state<CncCompletingRequest*>());
						state_ptr->set_event_handler(p);
						state_ptr = static_cast<BaseState*>(ref.get_state<GripperMoving*>());
						state_ptr->set_event_handler(p);
					}

					SubmachineInterface::set_event_handler(p);
				}

				// actions
				virtual void handle_event(robot_move_requested const &evnt)
				{

				}

				virtual void handle_event(robot_move_completed const &evnt)
				{

				}

				virtual void handle_event(robot_move_update const &evnt)
				{

				}

				virtual void handle_event(cnc_action_requested const &evnt)
				{

				}

				virtual void handle_event(cnc_action_completed const &evnt)
				{

				}

				virtual void handle_event(cnc_action_update const &evnt)
				{

				}

				virtual void handle_event(gripper_action_requested const &evnt)
				{

				}

				virtual void handle_event(gripper_action_completed const &evnt)
				{

				}

				virtual void handle_event(gripper_action_update const &evnt)
				{

				}

				virtual void handle_event(material_load_requested const &evnt)
				{

				}

				virtual void handle_event(material_unload_requested const &evnt)
				{

				}

				// transition table
				typedef MaterialHandling_ p;

				// exit pseudo states
				struct MaterialLoadCompleteExit : public boost::msm::front::exit_pseudo_state<material_load_completed>{};
				struct MaterialLoadFailureExit : public boost::msm::front::exit_pseudo_state<material_load_failed>{};
				struct MaterialUnloadCompleteExit : public boost::msm::front::exit_pseudo_state<material_unload_completed>{};
				struct MaterialUnloadFailureExit : public boost::msm::front::exit_pseudo_state<material_unload_failed>{};


				struct transition_table : boost::mpl::vector<
				 //    Start          Event                      Next                   Action				         Guard
				//  +----------------+--------------------------+----------------------+---------------------+----------------------------+
				a_row < BaseState    , material_load_requested  , BaseState    			,   &p::handle_event   										>,
				a_row < BaseState    , material_unload_requested, BaseState    			,   &p::handle_event   										>,
				a_row < BaseState    , robot_move_requested     , RobotMoving    		,   &p::handle_event   										>,
				a_row < BaseState    , cnc_action_requested     , CncCompletingRequest  ,   &p::handle_event 										>,
				a_row < BaseState    , gripper_action_requested , GripperMoving    		,   &p::handle_event 										>,
				//  +----------------+--------------------------+----------------------+---------------------+----------------------------+
				a_row < RobotMoving  , robot_move_update 		, BaseState     		,   &p::handle_event       									>,
				a_row < RobotMoving  , robot_move_completed 	, BaseState         	,   &p::handle_event       									>,
				//  +----------------+--------------------------+----------------------+---------------------+----------------------------+
				a_row < CncCompletingRequest, cnc_action_completed , BaseState     		,   &p::handle_event            							>,
				a_row < CncCompletingRequest, cnc_action_update   , BaseState     		,   &p::handle_event            							>,
				//  +----------------+--------------------------+----------------------+---------------------+----------------------------+
				a_row < GripperMoving , gripper_action_completed , BaseState     		,   &p::handle_event                    					>,
				a_row < GripperMoving , gripper_action_update    , BaseState     		,   &p::handle_event            							>,
				//  +----------------+--------------------------+----------------------+---------------------+----------------------------+
				_row < BaseState 	 , material_load_completed   , MaterialLoadCompleteExit     				           							>,
				_row < BaseState 	 , material_load_failed   	 , MaterialLoadFailureExit     				           								>,
				_row < BaseState 	 , material_unload_completed , MaterialUnloadCompleteExit     				           							>,
				_row < BaseState 	 , material_unload_failed    , MaterialUnloadFailureExit     				           							>
				>{};

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
					current_task_index_ = 0;
					ready_for_request_ = true;
					std::cout<< indenting_str_ <<"Leaving "<<name_<<" sub-machine\n";
				}

		        // no-transition response
		        template <class FSM,class Event>
					void no_transition(Event const& e, FSM& sm,int state)
				{
					states::BaseState* st =  static_cast<states::BaseState*>( sm.get_state_by_id(state));

					std::cout << "no transition from state "<< st->name_ << "( id "<<state<<")"
							<< " on event " << typeid(e).name()<< std::endl;
				}

			};

			// material load sub-machine front-end
			struct MaterialLoad_ : public MaterialHandling_
			{
				typedef boost::msm::back::state_machine<MaterialLoad_> BackendType;

				MaterialLoad_(std::string name = "Material Load"):
						MaterialHandling_(name)
				{
					// creating task sequence
					task_list_.clear();
					task_list_ = list_of((int)ROBOT_MOVE_HOME)
							((int)CNC_OPEN_DOOR)
							((int)CNC_OPEN_CHUCK)
							((int)ROBOT_PICKUP_MATERIAL)
							((int)ROBOT_APPROACH_CNC)
							((int)ROBOT_ENTER_CNC)
							((int)ROBOT_MOVE_TO_CHUCK)
							((int)CNC_CLOSE_CHUCK)
							((int)GRIPPER_OPEN)
							((int)ROBOT_RETREAT_FROM_CHUCK)
							((int)ROBOT_EXIT_CNC)
							((int)CNC_CLOSE_DOOR);

				}


				virtual bool verify_completion_and_proceed(int task_id)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);
					bool success = true;

					// checking completed task
					if(task_list_[current_task_index_++] == task_id)
					{
						if(current_task_index_ < (int)task_list_.size())
						{
							std::cout<<indenting_str_<<name_<<" completed task "<<current_task_index_ - 1<<" '"<<
									MaterialTaskDescriptionMap[task_id]<<"'"<<std::endl;

							std::cout<<indenting_str_<<name_<<" dispatching task "<<current_task_index_ <<" '"<<
									MaterialTaskDescriptionMap[task_list_[current_task_index_]]<<"'" <<std::endl;

							dispatch_task(task_list_[current_task_index_]);


						}
						else
						{
							// resetting variables
							current_task_index_ = 0;
							ready_for_request_ = true;

							// processing completion event
							std::cout<<indenting_str_<<name_<<" completed task "<<current_task_index_ - 1<<" '"<<
																MaterialTaskDescriptionMap[task_id]<<"'"<<std::endl;
							std::cout<<indenting_str_<<name_<<" completed all tasks"<< std::endl;

							ref.process_event(events::material_load_completed());
						}
					}
					else
					{
						success = false;
						std::cout<<indenting_str_<<name_<<" completed task failed validation, ending request"<< std::endl;

						//ref.process_event(events::material_load_failed());
					}

					return success;
				}

				virtual void handle_event(material_load_requested const &evnt)
				{
					std::cout<< indenting_str_ <<name_<<" sub-machine handling material load request\n";
					static BackendType &ref = static_cast<BackendType&>(*this);
					if(ready_for_request_)
					{
						std::cout<<indenting_str_<<name_<<" dispatching task "<<current_task_index_ <<" '"<<
								MaterialTaskDescriptionMap[task_list_[current_task_index_]]<<"'" <<std::endl;

						current_task_index_ = 0;
						ready_for_request_ = false;
						dispatch_task(task_list_[current_task_index_]);


					}
					else
					{
						std::cout<<indenting_str_<<name_<<" unable to handle request event, exiting"<< std::endl;
						ref.process_event(events::material_load_failed(BUSY_ERROR,ErrorTypeDescriptionMap[BUSY_ERROR]));
					}
				}

				virtual void handle_event(robot_move_completed const &evnt)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);

					if(!evnt.success_)
					{
						//ref.enqueue_event(events::robot_fault_detected());
						ref.process_event(events::material_load_failed(ROBOT_ERROR,ErrorTypeDescriptionMap[ROBOT_ERROR]));
					}
					else
					{
						if(!verify_completion_and_proceed(evnt.task_id_))
						{
							//ref.enqueue_event(events::robot_fault_detected());
							ref.process_event(events::material_load_failed(ROBOT_ERROR,ErrorTypeDescriptionMap[ROBOT_ERROR]));
						}
					}
				}

				virtual void handle_event(cnc_action_completed const &evnt)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);

					if(!evnt.success_)
					{
						//ref.enqueue_event(events::robot_fault_detected());
						ref.process_event(events::material_load_failed(CNC_ERROR,ErrorTypeDescriptionMap[CNC_ERROR]));
					}
					else
					{
						if(!verify_completion_and_proceed(evnt.task_id_))
						{
							//ref.enqueue_event(events::robot_fault_detected());
							ref.process_event(events::material_load_failed(CNC_ERROR,ErrorTypeDescriptionMap[CNC_ERROR]));
						}
					}
				}

				virtual void handle_event(gripper_action_completed const &evnt)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);

					if(!evnt.success_)
					{
						//ref.enqueue_event(events::robot_fault_detected());
						ref.process_event(events::material_load_failed(GRIPPER_ERROR,ErrorTypeDescriptionMap[GRIPPER_ERROR]));
					}
					else
					{
						if(!verify_completion_and_proceed(evnt.task_id_))
						{
							//ref.enqueue_event(events::robot_fault_detected());
							ref.process_event(events::material_load_failed(GRIPPER_ERROR,ErrorTypeDescriptionMap[GRIPPER_ERROR]));
						}
					}
				}

				// on entry method (generic)
				template <typename Event, typename FSM>
					void on_entry(Event const &evnt , FSM &sm)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);
					std::cout<< SubmachineInterface::indenting_str_ <<"Entering "<<SubmachineInterface::name_<<" sub-machine\n";

					handle_event(material_load_requested());

				}

				// on entry method (specialized to handle material load request events
					void on_entry(events::material_load_requested const &evnt , BackendType &sm)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);
					std::cout<< SubmachineInterface::indenting_str_ <<"Entering "<<SubmachineInterface::name_<<" sub-machine\n";

					handle_event(evnt);
				}
			};

			// material load sub-machine front-end
			struct MaterialUnload_ : public MaterialHandling_
			{
				typedef boost::msm::back::state_machine<MaterialUnload_> BackendType;

				MaterialUnload_(std::string name = "Material Unload"):
						MaterialHandling_(name)
				{
					// creating task sequence
					task_list_.clear();
					task_list_ = list_of((int)ROBOT_APPROACH_CNC)
							((int)GRIPPER_OPEN)
							((int)CNC_OPEN_DOOR)
							((int)ROBOT_ENTER_CNC)
							((int)ROBOT_MOVE_TO_CHUCK)
							((int)GRIPPER_CLOSE)
							((int)CNC_OPEN_CHUCK)
							((int)ROBOT_RETREAT_FROM_CHUCK)
							((int)ROBOT_EXIT_CNC)
							((int)ROBOT_PLACE_WORKPIECE)
							((int)CNC_CLOSE_CHUCK)
							((int)CNC_CLOSE_DOOR);

				}

				virtual bool verify_completion_and_proceed(int task_id)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);
					bool success = true;

					// checking completed task
					if(task_list_[current_task_index_++] == task_id)
					{
						if(current_task_index_ < (int)task_list_.size())
						{
							std::cout<<indenting_str_<<name_<<" completed task "<<current_task_index_ - 1<<" '"<<
									MaterialTaskDescriptionMap[task_id]<<"'"<<std::endl;

							std::cout<<indenting_str_<<name_<<" dispatching task "<<current_task_index_ <<" '"<<
									MaterialTaskDescriptionMap[task_list_[current_task_index_]]<<"'" <<std::endl;

							dispatch_task(task_list_[current_task_index_]);


						}
						else
						{
							// resetting variables
							current_task_index_ = 0;
							ready_for_request_ = true;

							// processing completion event
							std::cout<<indenting_str_<<name_<<" completed task "<<current_task_index_ - 1<<" '"<<
									MaterialTaskDescriptionMap[task_id]<<"'"<<std::endl;
							std::cout<<indenting_str_<<name_<<" completed all tasks"<< std::endl;

							ref.process_event(events::material_unload_completed());
						}
					}
					else
					{
						success = false;
						std::cout<<indenting_str_<<name_<<" completed task failed validation, ending request"<< std::endl;

						//ref.process_event(events::material_unload_failed());
					}

					return success;
				}

				virtual void handle_event(material_unload_requested const &evnt)
				{
					std::cout<< indenting_str_ <<name_<<" sub-machine handling material load request\n";
					static BackendType &ref = static_cast<BackendType&>(*this);
					if(ready_for_request_)
					{
						std::cout<<indenting_str_<<name_<<" dispatching task "<<current_task_index_ <<" '"<<
								MaterialTaskDescriptionMap[task_list_[current_task_index_]]<<"'" <<std::endl;

						current_task_index_ = 0;
						ready_for_request_ = false;
						dispatch_task(task_list_[current_task_index_]);
					}
					else
					{
						std::cout<<indenting_str_<<name_<<" unable to handle request event, exiting"<< std::endl;
						ref.process_event(events::material_unload_failed(BUSY_ERROR,ErrorTypeDescriptionMap[BUSY_ERROR]));
					}
				}

				virtual void handle_event(robot_move_completed const &evnt)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);

					if(!evnt.success_)
					{
						//ref.enqueue_event(events::robot_fault_detected());
						ref.process_event(events::material_unload_failed(ROBOT_ERROR,ErrorTypeDescriptionMap[ROBOT_ERROR]));
					}
					else
					{
						if(!verify_completion_and_proceed(evnt.task_id_))
						{
							//ref.enqueue_event(events::robot_fault_detected());
							ref.process_event(events::material_unload_failed(ROBOT_ERROR,ErrorTypeDescriptionMap[ROBOT_ERROR]));
						}
					}
				}

				virtual void handle_event(cnc_action_completed const &evnt)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);

					if(!evnt.success_)
					{
						//ref.enqueue_event(events::robot_fault_detected());
						ref.process_event(events::material_unload_failed(CNC_ERROR,ErrorTypeDescriptionMap[CNC_ERROR]));
					}
					else
					{
						if(!verify_completion_and_proceed(evnt.task_id_))
						{
							//ref.enqueue_event(events::robot_fault_detected());
							ref.process_event(events::material_unload_failed(CNC_ERROR,ErrorTypeDescriptionMap[CNC_ERROR]));
						}
					}
				}

				virtual void handle_event(gripper_action_completed const &evnt)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);

					if(!evnt.success_)
					{
						//ref.enqueue_event(events::robot_fault_detected());
						ref.process_event(events::material_unload_failed(GRIPPER_ERROR,ErrorTypeDescriptionMap[GRIPPER_ERROR]));
					}
					else
					{
						if(!verify_completion_and_proceed(evnt.task_id_))
						{
							//ref.enqueue_event(events::robot_fault_detected());
							ref.process_event(events::material_unload_failed(GRIPPER_ERROR,ErrorTypeDescriptionMap[GRIPPER_ERROR]));
						}
					}
				}

				// on entry method (generic)
				template <typename Event, typename FSM>
					void on_entry(Event const &evnt , FSM &sm)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);
					std::cout<< SubmachineInterface::indenting_str_ <<"Entering "<<SubmachineInterface::name_<<" sub-machine\n";
					handle_event(material_unload_requested());

				}

				// on entry method (specialized to handle material load request events
					void on_entry(events::material_unload_requested const &evnt , BackendType &sm)
				{
					static BackendType &ref = static_cast<BackendType&>(*this);
					std::cout<< SubmachineInterface::indenting_str_ <<"Entering "<<SubmachineInterface::name_<<" sub-machine\n";
					handle_event(evnt);
				}
			};

			//material load sub-machine back-end
			typedef boost::msm::back::state_machine<MaterialLoad_> MaterialLoad;
			typedef boost::msm::back::state_machine<MaterialUnload_> MaterialUnload;
			typedef boost::shared_ptr<MaterialLoad> MaterialLoadPtr;
			typedef boost::shared_ptr<MaterialUnload> MaterialUnloadPtr;
		}
	}
}

#endif /* MATERIAL_HANDLING_SUBMACHINE_H_ */
