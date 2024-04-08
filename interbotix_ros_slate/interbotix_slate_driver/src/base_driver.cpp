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

#include "interbotix_slate_driver/base_driver.hpp"
#include "interbotix_slate_driver/serial_driver.hpp"

namespace base_driver
{

SerialDriver * serial_driver = nullptr;

bool chassisInit(std::string & dev)
{
  serial_driver = new SerialDriver();
  return serial_driver->init(PORT, dev, 115200);
}

void exit()
{
  serial_driver->close();
  ::exit(-1);
}

bool getBatteryInfo(float & vol, float & cur, int & percent)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->getEntry(INDEX_SYS_VOLTAGE, &vol) &&
         serial_driver->getEntry(INDEX_SYS_CURRENT, &cur) &&
         serial_driver->getEntry(INDEX_SYS_POWER_PERCENTAGE, &percent);
}

bool getChassisCollision(int & collision)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->getEntry(INDEX_SYS_COLLISION, &collision);
}

bool getChassisState(SystemState & state)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->getEntry(INDEX_SYS_STATE, (int *)&state);  // NOLINT
}

bool getVersion(char * text)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->getEntry(INDEX_SYS_VERSION, text);
}

bool getJoyState(int & state)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->getEntry(INDEX_STATE_JOY, (int *)&state);  // NOLINT
}

bool setText(const char * text)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->setEntry(INDEX_SYS_TEXT, text);
}

bool setCharge(int charge)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->setEntry(INDEX_SYS_CHARGE, &charge);
}

bool setAlarm(int alarm)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->setEntry(INDEX_SYS_ALARM, &alarm);
}

bool motorCtrl(int v)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->setEntry(INDEX_SYS_CMD, &v);
}

bool setIo(int io)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->setEntry(INDEX_STATE_IO, &io);
}

bool getIo(int & io_state)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->getEntry(INDEX_STATE_IO, &io_state);
}

bool setLight(int light)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->setEntry(INDEX_SYS_LIGHT, &light);
}

bool setStateLight(int light)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->setEntry(INDEX_STATE_LIGHT, &light);
}

/**
 * @brief Set the body velocity
 * @param aim_x_vel forward velocity [m/s]
 * @param aim_z_omega rotational velocity [rad/s]
 */
bool chassisControl(float aim_x_vel, float aim_z_omega)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->setEntry(INDEX_AIM_CHASSIS_VEL, &aim_x_vel) &&
         serial_driver->setEntry(INDEX_AIM_CHASSIS_POS_OR_OMEGA, &aim_z_omega);
}

/**
 * @brief Get body velocity
 * @param x_vel m/s
 * @param z_omega rad/s
 * @return true if driver was able to retrieve requested values, false otherwise
 */
bool getChassisInfo(float & x_vel, float & z_omega)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->getEntry(INDEX_CHASSIS_VEL, &x_vel) &&
         serial_driver->getEntry(INDEX_CHASSIS_POS_OR_OMEGA, &z_omega);
}

/**
 * @brief Get chassis odometry
 * @param odom_x odom along the x-axis
 * @param odom_y odom along the y-axis
 * @param odom_theta odom around the z-axis
 * @return true if driver was able to retrieve requested values, false otherwise
 */
bool getChassisOdom(float & odom_x, float & odom_y, float & odom_theta)
{
  ASSERT(serial_driver != nullptr, exit());
  return serial_driver->getEntry(INDEX_CHASSIS_ODOM_X, &odom_x) &&
         serial_driver->getEntry(INDEX_CHASSIS_ODOM_Y, &odom_y) &&
         serial_driver->getEntry(INDEX_CHASSIS_ODOM_THETA, &odom_theta);
}

}  // namespace base_driver
