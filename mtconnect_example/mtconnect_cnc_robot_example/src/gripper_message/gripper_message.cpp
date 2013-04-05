/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef ROS
#include <mtconnect_cnc_robot_example/gripper_message/gripper_message.h>
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include <mtconnect_cnc_robot_example/gripper_message/gripper_message.h>
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace mtconnect_cnc_robot_example::gripper_message;

namespace mtconnect_cnc_robot_example
{
namespace gripper_message
{

GripperMessage::GripperMessage(void)
{
  this->setMessageType(industrial::simple_message::StandardMsgTypes::SWRI_MSG_BEGIN);
  this->init();
}

GripperMessage::~GripperMessage(void)
{

}

bool GripperMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();

  this->setMessageType(industrial::simple_message::StandardMsgTypes::SWRI_MSG_BEGIN);
  this->unload(&msg.getData());
  return rtn;

}

void GripperMessage::init(GripperOperationType operation)
{
  this->operation_ = operation;
}

void GripperMessage::init()
{
  this->operation_ = GripperOperationTypes::INVALID;
}

bool GripperMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing gripper message load");
  if (buffer->load(this->operation_))
  {
    rtn = true;
  }

  else
  {
    rtn = false;
    LOG_ERROR("Failed to load operation data");
  }
  return rtn;
}

bool GripperMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing gripper message unload");
  shared_int op_int;

  if (buffer->unload(op_int))
  {
    this->operation_ = (GripperOperationType)op_int;
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload operation data");
  }
  return rtn;
}

}
}

