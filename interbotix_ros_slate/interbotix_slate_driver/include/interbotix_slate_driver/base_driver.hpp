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

#ifndef INTERBOTIX_SLATE_DRIVER__BASE_DRIVER_HPP_
#define INTERBOTIX_SLATE_DRIVER__BASE_DRIVER_HPP_

#include <string>

#include "interbotix_slate_driver/od_index.hpp"

namespace base_driver
{

bool chassisInit(std::string & dev);
bool chassisControl(float aim_x_vel, float aim_z_omega);
bool getChassisInfo(float & x_vel, float & z_omega);
bool getChassisOdom(float & odom_x, float & odom_y, float & odom_theta);

bool getBatteryInfo(float & vol, float & cur, int & percent);
bool getChassisState(SystemState & state);
bool getChassisCollision(int & collision);
bool getVersion(char * text);
bool getJoyState(int & state);

// Limited to 100 characters
bool setText(const char * text);

// 0 / 1
bool setCharge(int charge);

// 0 / 1
bool setAlarm(int alarm);

bool motorCtrl(int v);

bool setIo(int io);

bool getIo(int & io_state);

// 0 ~ 100
bool setLight(int light);

bool setStateLight(int light);

}  // namespace base_driver

#endif  // INTERBOTIX_SLATE_DRIVER__BASE_DRIVER_HPP_
