#ifndef INTERBOTIX_SLATE_DRIVER__BASE_DRIVER_H_
#define INTERBOTIX_SLATE_DRIVER__BASE_DRIVER_H_

#include <string>

#include "od_index.h"

namespace base_driver
{

bool chassisInit(std::string &dev);
bool chassisControl(float aim_x_vel, float aim_z_omega);
bool getChassisInfo(float &x_vel, float &z_omega);
bool getChassisOdom(float &odom_x, float &odom_y, float &odom_theta);

bool getBatteryInfo(float &vol, float &cur, int &percent);
bool getChassisState(SystemState &state);
bool getChassisCollision(int &collision);
bool getVersion(char *text);
bool getJoyState(int &state);

// Limited to 100 characters
bool setText(const char *text);

// 0 / 1
bool setCharge(int charge);

// 0 / 1
bool setAlarm(int alarm);

bool motorCtrl(int v);

bool setIo(int io);

bool getIo(int &io_state);

// 0 ~ 100
bool setLight(int light);

bool setStateLight(int light);

}  // namespace base_driver

#endif  // INTERBOTIX_SLATE_DRIVER__BASE_DRIVER_H_
