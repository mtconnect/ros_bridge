/*
 * Copyright 2013 Southwest Research Institute

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#ifndef GRIPPER_MESSAGE_H
#define GRIPPER_MESSAGE_H

#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"


namespace mtconnect_cnc_robot_example
{
namespace gripper_message
{

/**
 * \brief Enumeration of gripper operations
 */
namespace GripperOperationTypes
{
  enum GripperOperationType
  {
    INVALID = 0,
    INIT,
    OPEN,
    CLOSE
  };
}
typedef GripperOperationTypes::GripperOperationType GripperOperationType;

/**
 * \brief Class encapsulated longhorn robot gripper message.
 */
//* GripperMessage
/**
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class GripperMessage : public industrial::typed_message::TypedMessage

{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  GripperMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~GripperMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a joint trajectory point structure
   *
   * \param gripper operation enumeration
   *
   */
  void init(GripperOperationType operation);

  /**
   * \brief Initializes a new message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int(operation_));
  }

  GripperOperationType operation_;

private:


};

}
}

#endif /* GRIPPER_MESSAGE_H */
