// Copyright 2024 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef INTERBOTIX_SLATE_DRIVER__OD_INDEX_HPP_
#define INTERBOTIX_SLATE_DRIVER__OD_INDEX_HPP_

#define PORT "chassis"

typedef enum
{
  SYS_INIT = 0x00,
  SYS_NORMAL,
  SYS_REMOTE,
  SYS_ESTOP,
  SYS_CALIB,
  SYS_TEST,
  SYS_CHARGING,

  SYS_ERR = 0x10,
  SYS_ERR_ID,
  SYS_ERR_COM,
  SYS_ERR_ENC,
  SYS_ERR_COLLISION,
  SYS_ERR_LOW_VOLTAGE,
  SYS_ERR_OVER_VOLTAGE,
  SYS_ERR_OVER_CURRENT,
  SYS_ERR_OVER_TEMP,

  SYS_STATE_LEN,
} SystemState;

typedef enum
{
  INDEX_SYS_STATE = 0,
  INDEX_SYS_POWER_PERCENTAGE = 1,

  INDEX_CHASSIS_VEL = 2,
  INDEX_CHASSIS_POS_OR_OMEGA = 3,
  INDEX_CHASSIS_ODOM_X = 4,
  INDEX_CHASSIS_ODOM_Y = 5,
  INDEX_CHASSIS_ODOM_THETA = 6,

  INDEX_SYS_VOLTAGE = 7,
  INDEX_SYS_CURRENT = 8,

  INDEX_AIM_CHASSIS_VEL = 9,
  INDEX_AIM_CHASSIS_POS_OR_OMEGA = 10,

  INDEX_SYS_CHARGE = 11,
  INDEX_SYS_ALARM = 12,
  INDEX_SYS_TEXT = 13,
  INDEX_SYS_LIGHT = 14,
  INDEX_SYS_COLLISION = 15,
  INDEX_SYS_VERSION = 16,
  INDEX_STATE_LIGHT = 17,
  INDEX_STATE_JOY = 18,
  INDEX_STATE_IO = 21,
  INDEX_SYS_CMD = 22,
} OdIndex;

#endif  // INTERBOTIX_SLATE_DRIVER__OD_INDEX_HPP_
