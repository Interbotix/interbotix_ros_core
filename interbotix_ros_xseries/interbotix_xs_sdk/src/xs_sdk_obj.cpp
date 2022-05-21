// Copyright 2022 Trossen Robotics
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
//    * Neither the name of the the copyright holder nor the names of its
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

#include <string>
#include <vector>
#include <memory>

#include "interbotix_xs_sdk/xs_sdk_obj.hpp"

namespace interbotix_xs
{

InterbotixRobotXS::InterbotixRobotXS(
  bool & success,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("xs_sdk", options)
{
  robot_init_parameters();
  if (!robot_get_motor_configs()) {
    success = false;
    return;
  }

  if (!robot_init_port()) {
    success = false;
    return;
  }

  if (!robot_ping_motors()) {
    success = false;
    RCLCPP_ERROR(LOGGER, "Failed to find all motors. Shutting down...");
    return;
  }

  if (!robot_load_motor_configs()) {
    success = false;
    RCLCPP_ERROR(
      LOGGER, "Failed to write configurations to all motors. Shutting down...");
    return;
  }

  robot_init_controlItems();
  robot_init_workbench_handlers();
  robot_init_operating_modes();
  robot_init_publishers();
  robot_init_subscribers();
  robot_init_services();
  robot_init_timers();
  robot_wait_for_joint_states();
  RCLCPP_INFO(LOGGER, "Interbotix 'xs_sdk' node is up!");
}

/// @brief Destructor for the InterbotixRobotXS
InterbotixRobotXS::~InterbotixRobotXS() {}

/// @brief Declare all parameters needed by the node
void InterbotixRobotXS::robot_init_parameters()
{
  this->declare_parameter<std::string>("motor_configs", "");
  this->declare_parameter<std::string>("mode_configs", "");
  this->declare_parameter<bool>("load_configs", false);
  this->declare_parameter<std::string>("robot_description", "");
}

void InterbotixRobotXS::robot_set_operating_modes(
  const std::string cmd_type,
  const std::string & name,
  const std::string & mode,
  const std::string profile_type,
  const int32_t profile_velocity,
  const int32_t profile_acceleration)
{
  if (cmd_type == CMD_TYPE_GROUP && group_map.count(name) > 0) {
    for (auto const & joint_name : group_map[name].joint_names) {
      robot_set_joint_operating_mode(
        joint_name,
        mode,
        profile_type,
        profile_velocity,
        profile_acceleration);
    }
    group_map[name].mode = mode;
    group_map[name].profile_type = profile_type;
    RCLCPP_INFO(
      LOGGER,
      "The operating mode for the '%s' group was changed to '%s' with profile type '%s'.",
      name.c_str(), mode.c_str(), profile_type.c_str());
  } else if (cmd_type == CMD_TYPE_SINGLE && motor_map.count(name) > 0) {
    robot_set_joint_operating_mode(
      name,
      mode,
      profile_type,
      profile_velocity,
      profile_acceleration);
    RCLCPP_INFO(
      LOGGER,
      "The operating mode for the '%s' joint was changed to '%s' with profile type '%s'.",
      name.c_str(), mode.c_str(), profile_type.c_str());
  } else if ( // NOLINT https://github.com/ament/ament_lint/issues/158
    (cmd_type == CMD_TYPE_GROUP && group_map.count(name) == 0) ||
    (cmd_type == CMD_TYPE_SINGLE && motor_map.count(name) == 0))
  {
    RCLCPP_WARN(
      LOGGER,
      "The '%s' joint/group does not exist. Was it added to the motor config file?",
      name.c_str());
  } else {
    RCLCPP_ERROR(
      LOGGER,
      "Invalid command for argument 'cmd_type' while setting operating mode.");
  }
}

void InterbotixRobotXS::robot_set_joint_operating_mode(
  const std::string & name,
  const std::string & mode,
  const std::string profile_type,
  const int32_t profile_velocity,
  const int32_t profile_acceleration)
{
  // torque off sister servos
  for (auto const & joint_name : sister_map[name]) {
    dxl_wb.torque(motor_map[joint_name].motor_id, false);
    RCLCPP_DEBUG(LOGGER, "ID: %d, torqued off.", motor_map[joint_name].motor_id);
  }

  for (auto const & motor_name : shadow_map[name]) {
    int32_t drive_mode;
    // read drive mode for each shadow
    dxl_wb.itemRead(motor_map[motor_name].motor_id, "Drive_Mode", &drive_mode);
    RCLCPP_DEBUG(
      LOGGER,
      "ID: %d, read Drive_Mode %d.",
      motor_map[motor_name].motor_id, drive_mode);
    // The 2nd (0x04) bit of the Drive_Mode register sets Profile Configuration
    // [0/false]: Velocity-based Profile: Create a Profile based on Velocity
    // [1/true]: Time-based Profile: Create Profile based on time
    std::bitset<8> drive_mode_bitset = drive_mode;
    if (profile_type == PROFILE_TIME) {
      drive_mode_bitset[2] = true;
    } else if (profile_type == PROFILE_TIME) {
      drive_mode_bitset[2] = false;
    }

    // write the correct drive mode based on the profile type
    // see https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drive-mode for details
    if (drive_mode <= 1 && profile_type == PROFILE_TIME) {
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Drive_Mode", drive_mode_bitset.to_ulong());
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, write Drive_Mode %ld.",
        motor_map[motor_name].motor_id, drive_mode_bitset.to_ulong());
    } else if (drive_mode >= 4 && profile_type == PROFILE_VELOCITY) {
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Drive_Mode", drive_mode_bitset.to_ulong());
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, write Drive_Mode %ld.",
        motor_map[motor_name].motor_id, drive_mode_bitset.to_ulong());
    }

    if (mode == MODE_POSITION || mode == MODE_LINEAR_POSITION) {
      // set position control mode if the mode is position or linear_position
      // also set prof_acc and prof_vel
      dxl_wb.setPositionControlMode(motor_map[motor_name].motor_id);
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Velocity",
        profile_velocity);
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Acceleration",
        profile_acceleration);
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, set poscontrolmode, pv=%i, pa=%i.",
        motor_map[motor_name].motor_id, profile_velocity, profile_acceleration);
    } else if (mode == MODE_EXT_POSITION) {
      // set ext_position control mode if the mode is ext_position
      // also set prof_acc and prof_vel
      dxl_wb.setExtendedPositionControlMode(
        motor_map[motor_name].motor_id);
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Velocity",
        profile_velocity);
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Acceleration",
        profile_acceleration);
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, set extposcontrolmode, pv=%i, pa=%i.",
        motor_map[motor_name].motor_id, profile_velocity, profile_acceleration);
    } else if (mode == MODE_VELOCITY) {
      // set velocity control mode if the mode is velocity
      // also set prof_acc
      dxl_wb.setVelocityControlMode(
        motor_map[motor_name].motor_id);
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Acceleration",
        profile_acceleration);
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, set velcontrolmode, pa=%i.",
        motor_map[motor_name].motor_id, profile_acceleration);
    } else if (mode == MODE_PWM) {
      // set pwm control mode if the mode is pwm
      dxl_wb.setPWMControlMode(
        motor_map[motor_name].motor_id);
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, set pwmcontrolmode.",
        motor_map[motor_name].motor_id);
    } else if (mode == MODE_CURRENT) {
      // set current control mode if the mode is current
      dxl_wb.setCurrentControlMode(
        motor_map[motor_name].motor_id);
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, set currentcontrolmode",
        motor_map[motor_name].motor_id);
    } else if (mode == MODE_CURRENT_BASED_POSITION) {
      // set current_based_position control mode if the mode is current_based_position
      dxl_wb.setCurrentBasedPositionControlMode(
        motor_map[motor_name].motor_id);
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, set currentcontrolmode",
        motor_map[motor_name].motor_id);
    } else {
      // mode was invalid
      RCLCPP_ERROR(
        LOGGER,
        "Invalid command for argument 'mode' while setting the operating mode for the %s motor.",
        motor_name.c_str());
      continue;
    }
    // set the mode and profile_type of each servo in the motor map
    motor_map[motor_name].mode = mode;
    motor_map[motor_name].profile_type = profile_type;
  }

  // torque on all sister servos
  for (auto const & joint_name : sister_map[name]) {
    dxl_wb.torque(motor_map[joint_name].motor_id, true);
    RCLCPP_DEBUG(
      LOGGER,
      "ID: %d, torqued on.",
      motor_map[joint_name].motor_id);
  }
}

void InterbotixRobotXS::robot_torque_enable(
  const std::string cmd_type,
  const std::string & name,
  const bool & enable)
{
  if (cmd_type == CMD_TYPE_GROUP && group_map.count(name) > 0) {
    // group case
    for (auto const & joint_name : group_map[name].joint_names) {
      // torque each servo in group
      dxl_wb.torque(motor_map[joint_name].motor_id, enable);
    }

    // log torque action
    if (enable) {
      RCLCPP_INFO(LOGGER, "The '%s' group was torqued on.", name.c_str());
    } else {
      RCLCPP_INFO(LOGGER, "The '%s' group was torqued off.", name.c_str());
    }
  } else if (cmd_type == CMD_TYPE_SINGLE && motor_map.count(name) > 0) {
    // single case
    // torque the single servo
    dxl_wb.torque(motor_map[name].motor_id, enable);

    // log torque action
    if (enable) {
      RCLCPP_INFO(LOGGER, "The '%s' joint was torqued on.", name.c_str());
    } else {
      RCLCPP_INFO(LOGGER, "The '%s' joint was torqued off.", name.c_str());
    }
  } else if ( // NOLINT https://github.com/ament/ament_lint/issues/158
    (cmd_type == CMD_TYPE_GROUP && group_map.count(name) == 0) ||
    (cmd_type == CMD_TYPE_SINGLE && motor_map.count(name) == 0))
  {
    // invalid name
    RCLCPP_WARN(
      LOGGER,
      "The '%s' joint/group does not exist. Was it added to the motor config file?",
      name.c_str());
  } else {
    // inavlid cmd_type
    RCLCPP_ERROR(
      LOGGER,
      "Invalid command for argument 'cmd_type' while torquing joints.");
  }
}

void InterbotixRobotXS::robot_reboot_motors(
  const std::string cmd_type,
  const std::string & name,
  const bool & enable,
  const bool & smart_reboot)
{
  std::vector<std::string> joints_to_torque;
  if (cmd_type == CMD_TYPE_GROUP && group_map.count(name) > 0) {
    // group case
    for (auto const & joint_name : group_map[name].joint_names) {
      // iterate through each joint in group
      if (smart_reboot) {
        // if smart_reboot, find the servos that are in an error status
        int32_t value = 0;
        const char * log;
        bool success = dxl_wb.itemRead(
          motor_map[joint_name].motor_id,
          "Hardware_Error_Status",
          &value, &log);
        if (success && value == 0) {
          continue;
        }
      }
      // reboot the servo
      dxl_wb.reboot(motor_map[joint_name].motor_id);
      RCLCPP_INFO(LOGGER, "The '%s' joint was rebooted.", joint_name.c_str());
      if (enable) {
        // add servo to joints_to_torque if enabled
        joints_to_torque.push_back(joint_name);
      }
    }
    if (!smart_reboot) {
      RCLCPP_INFO(LOGGER, "The '%s' group was rebooted.", name.c_str());
    }
  } else if (cmd_type == CMD_TYPE_SINGLE && motor_map.count(name) > 0) {
    // single case
    // reboot the single servo
    dxl_wb.reboot(motor_map[name].motor_id);
    RCLCPP_INFO(LOGGER, "The '%s' joint was rebooted.", name.c_str());
    if (enable) {
      // add servo to joints_to_torque if enabled
      joints_to_torque.push_back(name);
    }
  } else if ( // NOLINT https://github.com/ament/ament_lint/issues/158
    (cmd_type == CMD_TYPE_GROUP && group_map.count(name) == 0) ||
    (cmd_type == CMD_TYPE_SINGLE && motor_map.count(name) == 0))
  {
    // invalid name
    RCLCPP_WARN(
      LOGGER,
      "The '%s' joint/group does not exist. Was it added to the motor config file?",
      name.c_str());
  } else {
    // invalid cmd_type
    RCLCPP_ERROR(
      LOGGER,
      "Invalid command for argument 'cmd_type' while rebooting motors.");
  }

  // torque servos in joints_to_torque and their sisters
  for (const auto & joint_name : joints_to_torque) {
    for (const auto & name : sister_map[joint_name]) {
      robot_torque_enable(CMD_TYPE_SINGLE, name, true);
    }
  }
}

void InterbotixRobotXS::robot_write_commands(
  const std::string & name,
  std::vector<float> commands)
{
  if (commands.size() != group_map[name].joint_num) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Number of commands (%ld) does not match the number of joints in group '%s' (%d). "
      "Will not execute.",
      commands.size(), name.c_str(), group_map[name].joint_num);
    return;
  }
  const std::string mode = group_map[name].mode;
  std::vector<int32_t> dynamixel_commands(commands.size());
  if (
    (mode == MODE_POSITION) ||
    (mode == MODE_EXT_POSITION) ||
    (mode == MODE_CURRENT_BASED_POSITION) ||
    (mode == MODE_LINEAR_POSITION))
  {
    // position commands case
    for (size_t i{0}; i < commands.size(); i++) {
      if (mode == MODE_LINEAR_POSITION) {
        // convert from linear position if necessary
        commands.at(i) = robot_convert_linear_position_to_radian(
          group_map[name].joint_names.at(i), commands.at(i));
      }
      // translate from position to command value
      dynamixel_commands[i] = dxl_wb.convertRadian2Value(
        group_map[name].joint_ids.at(i), commands.at(i));
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, writing %s command %d.",
        group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    // write position commands
    dxl_wb.syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, group_map[name].joint_ids.data(),
      group_map[name].joint_num, &dynamixel_commands[0], 1);
  } else if (mode == MODE_VELOCITY) {
    // velocity commands case
    for (size_t i{0}; i < commands.size(); i++) {
      // translate from velocity to command value
      dynamixel_commands[i] = dxl_wb.convertVelocity2Value(
        group_map[name].joint_ids.at(i), commands.at(i));
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, writing %s command %d.",
        group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    // write velocity commands
    dxl_wb.syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, group_map[name].joint_ids.data(),
      group_map[name].joint_num, &dynamixel_commands[0], 1);
  } else if (mode == MODE_CURRENT) {
    // velocity commands case
    for (size_t i{0}; i < commands.size(); i++) {
      // translate from current to command value
      dynamixel_commands[i] = dxl_wb.convertCurrent2Value(commands.at(i));
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, writing %s command %d.",
        group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    // write velocity commands
    dxl_wb.syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, group_map[name].joint_ids.data(),
      group_map[name].joint_num, &dynamixel_commands[0], 1);
  } else if (mode == MODE_PWM) {
    // pwm commands case, don't need to translate from pwm to value
    for (size_t i{0}; i < commands.size(); i++) {
      dynamixel_commands[i] = int32_t(commands.at(i));
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, writing %s command %d.",
        group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    // write pwm commands
    dxl_wb.syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_PWM, group_map[name].joint_ids.data(),
      group_map[name].joint_num, &dynamixel_commands[0], 1);
  } else {
    // invalid mode
    RCLCPP_ERROR(
      LOGGER,
      "Invalid command for argument 'mode' while commanding joint group.");
  }
}

void InterbotixRobotXS::robot_write_joint_command(
  const std::string & name,
  float command)
{
  const std::string mode = motor_map[name].mode;
  if (
    (mode == MODE_POSITION) ||
    (mode == MODE_EXT_POSITION) ||
    (mode == MODE_CURRENT_BASED_POSITION) ||
    (mode == MODE_LINEAR_POSITION))
  {
    // position command case
    if (mode == MODE_LINEAR_POSITION) {
      // convert from linear position if necessary
      command = robot_convert_linear_position_to_radian(name, command);
    }
    RCLCPP_DEBUG(
      LOGGER,
      "ID: %d, writing %s command %f.",
      motor_map[name].motor_id, mode.c_str(), command);
    // write position command
    dxl_wb.goalPosition(motor_map[name].motor_id, command);
  } else if (mode == MODE_VELOCITY) {
    // position velocity case
    RCLCPP_DEBUG(
      LOGGER,
      "ID: %d, writing %s command %f.",
      motor_map[name].motor_id, mode.c_str(), command);
    // write velocity command
    dxl_wb.goalVelocity(motor_map[name].motor_id, command);
  } else if (mode == MODE_CURRENT) {
    // position current case
    RCLCPP_DEBUG(
      LOGGER,
      "ID: %d, writing %s command %f.",
      motor_map[name].motor_id, mode.c_str(), command);
    // write current command
    dxl_wb.itemWrite(
      motor_map[name].motor_id, "Goal_Current", dxl_wb.convertCurrent2Value(command));
  } else if (mode == MODE_PWM) {
    // pwm current case
    RCLCPP_DEBUG(
      LOGGER,
      "ID: %d, writing %s command %f.", motor_map[name].motor_id, mode.c_str(), command);
    // write pwm command
    dxl_wb.itemWrite(motor_map[name].motor_id, "Goal_PWM", int32_t(command));
  } else {
    // invalid mode
    RCLCPP_ERROR(
      LOGGER,
      "Invalid command for argument 'mode' while commanding joint.");
  }
}

void InterbotixRobotXS::robot_set_motor_pid_gains(
  const std::string cmd_type,
  const std::string & name,
  const std::vector<int32_t> & gains)
{
  std::vector<std::string> names;
  if (cmd_type == CMD_TYPE_GROUP) {
    // group case, get names from group map
    names = group_map[name].joint_names;
  } else if (cmd_type == CMD_TYPE_SINGLE) {
    // single case, just use given name
    names.push_back(name);
  }

  for (auto const & name : names) {
    // write gains for each servo
    uint8_t id = motor_map[name].motor_id;
    RCLCPP_DEBUG(LOGGER, "ID: %d, writing gains:", motor_map[name].motor_id);
    RCLCPP_DEBUG(LOGGER, "        Pos_P: %i", gains.at(0));
    RCLCPP_DEBUG(LOGGER, "        Pos_I: %i", gains.at(1));
    RCLCPP_DEBUG(LOGGER, "        Pos_D: %i", gains.at(2));
    RCLCPP_DEBUG(LOGGER, "        FF_1: %i", gains.at(3));
    RCLCPP_DEBUG(LOGGER, "        FF_2: %i", gains.at(4));
    RCLCPP_DEBUG(LOGGER, "        Vel_P: %i", gains.at(5));
    RCLCPP_DEBUG(LOGGER, "        Vel_I: %i", gains.at(6));
    dxl_wb.itemWrite(id, "Position_P_Gain", gains.at(0));
    dxl_wb.itemWrite(id, "Position_I_Gain", gains.at(1));
    dxl_wb.itemWrite(id, "Position_D_Gain", gains.at(2));
    dxl_wb.itemWrite(id, "Feedforward_1st_Gain", gains.at(3));
    dxl_wb.itemWrite(id, "Feedforward_2nd_Gain", gains.at(4));
    dxl_wb.itemWrite(id, "Velocity_P_Gain", gains.at(5));
    dxl_wb.itemWrite(id, "Velocity_I_Gain", gains.at(6));
  }
}

void InterbotixRobotXS::robot_set_motor_registers(
  const std::string cmd_type,
  const std::string & name,
  const std::string & reg,
  const int32_t & value)
{
  std::vector<std::string> names;
  if (cmd_type == CMD_TYPE_GROUP) {
    // group case, get names from group map
    names = group_map[name].joint_names;
  } else if (cmd_type == CMD_TYPE_SINGLE) {
    // single case, just use given name
    names.push_back(name);
  }

  for (auto const & name : names) {
    // write register for each servo
    RCLCPP_DEBUG(
      LOGGER,
      "ID: %d, writing reg: %s, value: %d.",
      motor_map[name].motor_id, reg.c_str(), value);
    dxl_wb.itemWrite(motor_map[name].motor_id, reg.c_str(), value);
  }
}

void InterbotixRobotXS::robot_get_motor_registers(
  const std::string cmd_type,
  const std::string & name,
  const std::string & reg,
  std::vector<int32_t> & values)
{
  std::vector<std::string> names;
  if (cmd_type == CMD_TYPE_GROUP) {
    // group case, get names from group map
    names = group_map[name].joint_names;
  } else if (cmd_type == CMD_TYPE_SINGLE) {
    // single case, just use given name
    names.push_back(name);
  }

  // get info on the register that is going to be read from
  const ControlItem * goal_reg = dxl_wb.getItemInfo(motor_map[names.front()].motor_id, reg.c_str());
  if (goal_reg == NULL) {
    RCLCPP_ERROR(
      LOGGER,
      "Could not get '%s' Item Info. Did you spell the register name correctly?",
      reg.c_str());
    return;
  }

  for (auto const & name : names) {
    int32_t value = 0;
    const char * log;
    // read register for each servo and check result for success
    if (!dxl_wb.itemRead(motor_map[name].motor_id, reg.c_str(), &value, &log)) {
      RCLCPP_ERROR(LOGGER, "%s", log);
      return;
    } else {
      RCLCPP_DEBUG(
        LOGGER,
        "ID: %d, reading reg: %s, value: %d.", motor_map[name].motor_id, reg.c_str(), value);
    }

    // add register to values vector with type depending on data_length
    if (goal_reg->data_length == 1) {
      values.push_back((uint8_t)value);
    } else if (goal_reg->data_length == 2) {
      values.push_back((int16_t)value);
    } else {
      values.push_back(value);
    }
  }
}

void InterbotixRobotXS::robot_get_joint_states(
  const std::string & name,
  std::vector<float> * positions,
  std::vector<float> * velocities,
  std::vector<float> * effort)
{
  for (const auto & joint_name : group_map[name].joint_names) {
    // iterate through each joint in group, reading pos, vel, and eff
    if (positions) {
      positions->push_back(joint_states.position.at(js_index_map[joint_name]));
    }
    if (velocities) {
      velocities->push_back(joint_states.velocity.at(js_index_map[joint_name]));
    }
    if (effort) {
      effort->push_back(joint_states.effort.at(js_index_map[joint_name]));
    }
    RCLCPP_DEBUG(LOGGER, "ID: %ld, got joint state.", js_index_map[joint_name]);
  }
}

void InterbotixRobotXS::robot_get_joint_state(
  const std::string & name,
  float * position,
  float * velocity,
  float * effort)
{
  // read pos, vel, and eff for specified joint
  if (position) {
    *position = joint_states.position.at(js_index_map[name]);
  }
  if (velocity) {
    *velocity = joint_states.velocity.at(js_index_map[name]);
  }
  if (effort) {
    *effort = joint_states.effort.at(js_index_map[name]);
  }
  RCLCPP_DEBUG(LOGGER, "ID: %ld, got joint state.", js_index_map[name]);
}

float InterbotixRobotXS::robot_convert_linear_position_to_radian(
  const std::string & name,
  const float & linear_position)
{
  float half_dist = linear_position / 2.0;
  float arm_length = gripper_map[name].arm_length;
  float horn_radius = gripper_map[name].horn_radius;

  // (pi / 2) - acos(horn_rad^2 + (pos / 2)^2 - arm_length^2) / (2 * horn_rad * (pos / 2))
  return 3.14159 / 2.0 - \
         acos(
    (pow(horn_radius, 2) + \
    pow(half_dist, 2) - \
    pow(arm_length, 2)) / (2 * horn_radius * half_dist));
}

float InterbotixRobotXS::robot_convert_angular_position_to_linear(
  const std::string & name,
  const float & angular_position)
{
  float arm_length = gripper_map[name].arm_length;
  float horn_radius = gripper_map[name].horn_radius;
  float a1 = horn_radius * sin(angular_position);
  float c = sqrt(pow(horn_radius, 2) - pow(a1, 2));
  float a2 = sqrt(pow(arm_length, 2) - pow(c, 2));
  return a1 + a2;
}

bool InterbotixRobotXS::robot_get_motor_configs()
{
  std::string motor_configs_file, mode_configs_file;

  // read motor_configs param
  this->get_parameter("motor_configs", motor_configs_file);
  try {
    // try to load motor_configs yaml file
    motor_configs = YAML::LoadFile(motor_configs_file.c_str());
  } catch (YAML::BadFile & error) {
    // if file is not found or a bad format, shut down
    RCLCPP_ERROR(
      LOGGER,
      "Motor Config file was not found or has a bad format. Shutting down...");
    RCLCPP_ERROR(LOGGER, "YAML Error: '%s'", error.what());
    return false;
  }

  if (motor_configs.IsNull()) {
    // if motor_configs is not found or empty, shut down
    RCLCPP_ERROR(LOGGER, "Motor Config file was not found. Shutting down...");
    return false;
  }

  // read mode_configs param
  this->get_parameter("mode_configs", mode_configs_file);
  try {
    // try to load mode_configs yaml file
    mode_configs = YAML::LoadFile(mode_configs_file.c_str());
    RCLCPP_INFO(
      LOGGER,
      "Loaded mode configs from '%s'.",
      mode_configs_file.c_str());
  } catch (YAML::BadFile & error) {
    // if file is not found or a bad format, shut down
    RCLCPP_ERROR(
      LOGGER,
      "Mode Config file was not found or has a bad format. Shutting down...");
    RCLCPP_ERROR(
      LOGGER,
      "YAML Error: '%s'",
      error.what());
    return false;
  }

  if (mode_configs.IsNull()) {
    // if mode_configs is not found or empty, use defaults
    RCLCPP_INFO(LOGGER, "Mode Config file is empty.");
  }

  // use specified or default port
  port = motor_configs["port"].as<std::string>(DEFAULT_PORT);
  if (mode_configs["port"]) {
    port = mode_configs["port"].as<std::string>(DEFAULT_PORT);
  }

  // create all_motors node from 'motors'
  YAML::Node all_motors = motor_configs["motors"];
  for (
    YAML::const_iterator motor_itr = all_motors.begin();
    motor_itr != all_motors.end();
    motor_itr++)
  {
    // iterate through each motor in all_motors node
    // get motor name
    std::string motor_name = motor_itr->first.as<std::string>();
    // create single_motor node from the motor_name
    YAML::Node single_motor = all_motors[motor_name];
    // extract ID from node
    uint8_t id = (uint8_t)single_motor["ID"].as<int32_t>();
    // add the motor to the motor_map with it's ID, pos as the default opmode, vel as default
    //  profile
    motor_map.insert({motor_name, {id, MODE_POSITION, PROFILE_VELOCITY}});
    for (
      YAML::const_iterator info_itr = single_motor.begin();
      info_itr != single_motor.end();
      info_itr++)
    {
      // iterate through the single_motor node
      // save all registers that are not ID or Baud_Rate
      std::string reg = info_itr->first.as<std::string>();
      if (reg != "ID" && reg != "Baud_Rate") {
        int32_t value = info_itr->second.as<int32_t>();
        MotorInfo motor_info = {id, reg, value};
        motor_info_vec.push_back(motor_info);
      }
    }
  }

  // create all_grippers node from 'grippers'
  YAML::Node all_grippers = motor_configs["grippers"];
  for (
    YAML::const_iterator gripper_itr = all_grippers.begin();
    gripper_itr != all_grippers.end();
    gripper_itr++)
  {
    // iterate through each gripper in all_grippers node
    // get gripper name
    std::string gripper_name = gripper_itr->first.as<std::string>();
    // create single_gripper node from the gripper_name
    YAML::Node single_gripper = all_grippers[gripper_name];
    // initialize a Gripper struct to save the info about this gripper
    Gripper gripper;
    // load all info from the single_gripper node into the Griper struct, substituting the default
    //  values if not given the value
    gripper.horn_radius = single_gripper["horn_radius"].as<float>(0.014);
    gripper.arm_length = single_gripper["arm_length"].as<float>(0.024);
    gripper.left_finger = single_gripper["left_finger"].as<std::string>("left_finger");
    gripper.right_finger = single_gripper["right_finger"].as<std::string>("right_finger");
    gripper_map.insert({gripper_name, gripper});
  }

  // create joint_order node from 'joint_order'
  YAML::Node joint_order = motor_configs["joint_order"];
  // create sleep_positions node from 'sleep_positions'
  YAML::Node sleep_positions = motor_configs["sleep_positions"];
  // create JointGroup struct for all_joints - contains all joints in robot
  JointGroup all_joints;
  all_joints.joint_num = (uint8_t) joint_order.size();
  all_joints.mode = MODE_POSITION;
  all_joints.profile_type = PROFILE_VELOCITY;
  for (size_t i{0}; i < joint_order.size(); i++) {
    // iterate through each joint in joint_order list
    // save each joint ID and name
    std::string joint_name = joint_order[i].as<std::string>();
    all_joints.joint_names.push_back(joint_name);
    all_joints.joint_ids.push_back(motor_map[joint_name].motor_id);
    // add this joint's index to the js_index_map
    js_index_map.insert({joint_name, i});
    // add any shadows
    shadow_map.insert({joint_name, {joint_name}});
    // add any sisters
    sister_map.insert({joint_name, {joint_name}});
    // add the sleep position of this joint, defaults to 0 if not specified
    sleep_map.insert({joint_name, sleep_positions[i].as<float>(0)});
    // if this joint is a gripper, add it to the gripper_name
    if (gripper_map.count(joint_name) > 0) {
      gripper_map[joint_name].js_index = i;
      gripper_order.push_back(joint_name);
    }
  }

  // append the left and right finger to the gripper_map
  for (auto const & name : gripper_order) {
    js_index_map.insert({gripper_map[name].left_finger, js_index_map.size()});
    js_index_map.insert({gripper_map[name].right_finger, js_index_map.size()});
  }

  // all the all_joints JointGroup to the group_map
  group_map.insert({"all", all_joints});
  // create a pointer from the group_map's all group reference
  all_ptr = &group_map["all"];

  // create all_shadows node from 'shadows'
  YAML::Node all_shadows = motor_configs["shadows"];
  for (
    YAML::const_iterator master_itr = all_shadows.begin();
    master_itr != all_shadows.end();
    master_itr++)
  {
    // iterate through each shadow servo's master in all_shadows node
    std::string master_name = master_itr->first.as<std::string>();
    YAML::Node master = all_shadows[master_name];
    YAML::Node shadow_list = master["shadow_list"];
    for (size_t i{0}; i < shadow_list.size(); i++) {
      shadow_map[master_name].push_back(shadow_list[i].as<std::string>());
    }
  }

  // create all_sisters node from 'sisters'
  YAML::Node all_sisters = motor_configs["sisters"];
  for (
    YAML::const_iterator sister_itr = all_sisters.begin();
    sister_itr != all_sisters.end();
    sister_itr++)
  {
    // iterate through each sister servo in all_sisters node
    // save each 2-in-1 servo to the sister_map
    std::string sister_one = sister_itr->first.as<std::string>();
    std::string sister_two = sister_itr->second.as<std::string>();
    sister_map[sister_one].push_back(sister_two);
    sister_map[sister_two].push_back(sister_one);
  }

  // create all_groups node from 'groups'
  YAML::Node all_groups = motor_configs["groups"];
  for (
    YAML::const_iterator group_itr = all_groups.begin();
    group_itr != all_groups.end();
    group_itr++)
  {
    // iterate through each sister servo in all_sisters node
    // get the name of the group
    std::string name = group_itr->first.as<std::string>();
    // get the list of joints in the group from the all_group map
    YAML::Node joint_list = all_groups[name];
    // create a JointGroup for this group
    JointGroup group;
    // the number of joints in this group is the size of the list of joints
    group.joint_num = (uint8_t) joint_list.size();
    for (size_t i{0}; i < joint_list.size(); i++) {
      // add each joint's name and id to the JointGroup
      std::string joint_name = joint_list[i].as<std::string>();
      group.joint_names.push_back(joint_name);
      group.joint_ids.push_back(motor_map[joint_name].motor_id);
    }
    // add the JointGroup and its name to the group_map
    group_map.insert({name, group});
  }

  // get the one-off configs, substituting the default if not specified
  YAML::Node pub_configs = motor_configs["joint_state_publisher"];
  timer_hz = pub_configs["update_rate"].as<int>(100);
  pub_states = pub_configs["publish_states"].as<bool>(true);
  js_topic = pub_configs["topic_name"].as<std::string>("joint_states");

  RCLCPP_INFO(
    LOGGER,
    "Loaded motor configs from '%s'.",
    motor_configs_file.c_str());
  return true;
}

bool InterbotixRobotXS::robot_init_port()
{
  // try to connect to the specified port at the default baudrate
  if (!dxl_wb.init(port.c_str(), DEFAULT_BAUDRATE)) {
    // if the connection fails, shut down
    RCLCPP_ERROR(
      LOGGER,
      "Failed to open port at '%s'. Shutting down...",
      port.c_str());
    return false;
  }
  return true;
}

bool InterbotixRobotXS::robot_ping_motors()
{
  for (const auto &[motor_name, motor_state] : motor_map) {
    // iterate through each servo in the motor_map
    uint16_t model_number = 0;
    // try to ping the servo
    if (!dxl_wb.ping(motor_state.motor_id, &model_number)) {
      // if any ping is unsuccessful, shut down
      RCLCPP_ERROR(
        LOGGER,
        "Can't find Dynamixel ID '%d',\tJoint Name : '%s'",
        motor_state.motor_id, motor_name.c_str());
      return false;
    } else {
      RCLCPP_INFO(
        LOGGER,
        "Found Dynamixel ID : %d,\tModel Number : %d,\tJoint Name : %s",
        motor_state.motor_id, model_number, motor_name.c_str());
    }
    // untorque each pinged servo, need to write data to the EEPROM
    dxl_wb.torque(motor_state.motor_id, false);
  }
  return true;
}

bool InterbotixRobotXS::robot_load_motor_configs()
{
  bool load_configs;
  this->get_parameter("load_configs", load_configs);

  // write info to each motor if load_configs param is true
  if (load_configs) {
    for (auto const & motor_info : motor_info_vec) {
      if (!dxl_wb.itemWrite(motor_info.motor_id, motor_info.reg.c_str(), motor_info.value)) {
        RCLCPP_ERROR(
          LOGGER,
          "[xs_sdk] Failed to write value[%d] on items[%s] to [ID : %d]",
          motor_info.value, motor_info.reg.c_str(), motor_info.motor_id);
        return false;
      }
    }
  } else {
    RCLCPP_INFO(LOGGER, "Skipping Load Configs...");
  }
  return true;
}

void InterbotixRobotXS::robot_init_controlItems()
{
  uint8_t motor_id = motor_map.begin()->second.motor_id;

  const ControlItem * goal_position = dxl_wb.getItemInfo(motor_id, "Goal_Position");
  if (!goal_position) {
    RCLCPP_ERROR(LOGGER, "Could not get 'Goal_Position' Item Info");
  }

  const ControlItem * goal_velocity = dxl_wb.getItemInfo(motor_id, "Goal_Velocity");
  if (!goal_velocity) {
    goal_velocity = dxl_wb.getItemInfo(motor_id, "Moving_Speed");
  }
  if (!goal_velocity) {
    RCLCPP_ERROR(LOGGER, "Could not get 'Goal_Velocity' or 'Moving_Speed' Item Info");
  }

  const ControlItem * goal_current = NULL;
  for (auto const & [_, motor_info] : motor_map) {
    goal_current = dxl_wb.getItemInfo(motor_info.motor_id, "Goal_Current");
    if (goal_current) {
      break;
    }
  }

  if (!goal_current) {
    RCLCPP_INFO(
      LOGGER,
      "Could not get 'Goal_Current' Item Info. This message can be "
      "ignored if none of the robot's motors support current control.");
  }

  const ControlItem * goal_pwm = dxl_wb.getItemInfo(motor_id, "Goal_PWM");
  if (!goal_pwm) {
    RCLCPP_ERROR(LOGGER, "Could not get 'Goal_PWM' Item Info");
  }

  const ControlItem * present_position = dxl_wb.getItemInfo(motor_id, "Present_Position");
  if (!present_position) {
    RCLCPP_ERROR(LOGGER, "Could not get 'Present_Position' Item Info");
  }

  const ControlItem * present_velocity = dxl_wb.getItemInfo(motor_id, "Present_Velocity");
  if (!present_velocity) {
    present_velocity = dxl_wb.getItemInfo(motor_id, "Present_Speed");
  }
  if (!present_velocity) {
    RCLCPP_ERROR(
      LOGGER,
      "Could not get 'Present_Velocity' or 'Present_Speed' Item Info");
  }

  const ControlItem * present_current = dxl_wb.getItemInfo(motor_id, "Present_Current");
  if (!present_current) {
    present_current = dxl_wb.getItemInfo(motor_id, "Present_Load");
  }
  if (!present_current) {
    RCLCPP_ERROR(
      LOGGER,
      "Could not get 'Present_Current' or 'Present_Load' Item Info");
  }

  control_items["Goal_Position"] = goal_position;
  control_items["Goal_Velocity"] = goal_velocity;
  control_items["Goal_Current"] = goal_current;
  control_items["Goal_PWM"] = goal_pwm;

  control_items["Present_Position"] = present_position;
  control_items["Present_Velocity"] = present_velocity;
  control_items["Present_Current"] = present_current;
}

void InterbotixRobotXS::robot_init_workbench_handlers()
{
  if (
    !dxl_wb.addSyncWriteHandler(
      control_items["Goal_Position"]->address, control_items["Goal_Position"]->data_length))
  {
    RCLCPP_ERROR(LOGGER, "Failed to add SyncWriteHandler for Goal_Position.");
  }

  if (
    !dxl_wb.addSyncWriteHandler(
      control_items["Goal_Velocity"]->address, control_items["Goal_Velocity"]->data_length))
  {
    RCLCPP_ERROR(LOGGER, "Failed to add SyncWriteHandler for Goal_Velocity.");
  }

  // only add a SyncWriteHandler for 'Goal_Current' if the register actually exists!
  if (control_items["Goal_Current"]) {
    if (
      !dxl_wb.addSyncWriteHandler(
        control_items["Goal_Current"]->address, control_items["Goal_Current"]->data_length))
    {
      RCLCPP_ERROR(LOGGER, "Failed to add SyncWriteHandler for Goal_Current.");
    }
  } else {
    RCLCPP_INFO(
      LOGGER,
      "SyncWriteHandler for Goal_Current not added as it's not supported.");
  }

  if (
    !dxl_wb.addSyncWriteHandler(
      control_items["Goal_PWM"]->address, control_items["Goal_PWM"]->data_length))
  {
    RCLCPP_ERROR(LOGGER, "Failed to add SyncWriteHandler for Goal_PWM.");
  }

  if (dxl_wb.getProtocolVersion() == 2.0f) {
    uint16_t start_address = std::min(
      control_items["Present_Position"]->address,
      control_items["Present_Current"]->address);
    /*
      As some models have an empty space between Present_Velocity and Present Current, read_length
      is modified as below.
    */
    // uint16_t read_length = control_items["Present_Position"]->data_length \
    //   + control_items["Present_Velocity"]->data_length \
    //   + control_items["Present_Current"]->data_length;
    uint16_t read_length = control_items["Present_Position"]->data_length + \
      control_items["Present_Velocity"]->data_length + \
      control_items["Present_Current"]->data_length + \
      2;
    if (!dxl_wb.addSyncReadHandler(start_address, read_length)) {
      RCLCPP_ERROR(LOGGER, "Failed to add SyncReadHandler");
    }
  }
}

void InterbotixRobotXS::robot_init_operating_modes()
{
  YAML::Node all_shadows = motor_configs["shadows"];
  for (
    YAML::const_iterator master_itr = all_shadows.begin();
    master_itr != all_shadows.end();
    master_itr++)
  {
    std::string master_name = master_itr->first.as<std::string>();
    YAML::Node master = all_shadows[master_name];
    if (master["calibrate"].as<bool>(false)) {
      int32_t master_position;
      dxl_wb.itemRead(motor_map[master_name].motor_id, "Present_Position", &master_position);
      for (auto const & shadow_name : shadow_map[master_name]) {
        if (shadow_name == master_name) {continue;}
        dxl_wb.itemWrite(motor_map[shadow_name].motor_id, "Homing_Offset", 0);
        int32_t shadow_position, shadow_drive_mode;
        dxl_wb.itemRead(motor_map[shadow_name].motor_id, "Present_Position", &shadow_position);
        dxl_wb.itemRead(motor_map[shadow_name].motor_id, "Drive_Mode", &shadow_drive_mode);
        // The 0th (0x01) bit of the Drive_Mode register sets Normal/Reverse Mode
        // [0/false]: Normal Mode: CCW(Positive), CW(Negative)
        // [1/true]: Reverse Mode: CCW(Negative), CW(Positive)
        // This mode dictates how to calculate the homing offset of the shadow motor
        std::bitset<8> shadow_drive_mode_bitset = shadow_drive_mode;
        int32_t homing_offset;
        if (shadow_drive_mode_bitset[0]) {
          homing_offset = master_position - shadow_position;
        } else {
          homing_offset = shadow_position - master_position;
        }
        dxl_wb.itemWrite(motor_map[shadow_name].motor_id, "Homing_Offset", homing_offset);
      }
    }
  }

  YAML::Node all_groups = mode_configs["groups"];
  for (
    YAML::const_iterator group_itr = all_groups.begin();
    group_itr != all_groups.end();
    group_itr++)
  {
    std::string name = group_itr->first.as<std::string>();
    YAML::Node single_group = all_groups[name];
    const std::string operating_mode =
      single_group["operating_mode"].as<std::string>(DEFAULT_OP_MODE);
    const std::string profile_type =
      single_group["profile_type"].as<std::string>(DEFAULT_PROF_TYPE);
    int32_t profile_velocity = single_group["profile_velocity"].as<int32_t>(DEFAULT_PROF_VEL);
    int32_t profile_acceleration =
      single_group["profile_acceleration"].as<int32_t>(DEFAULT_PROF_ACC);
    robot_set_operating_modes(
      CMD_TYPE_GROUP,
      name,
      operating_mode,
      profile_type,
      profile_velocity,
      profile_acceleration);
    if (!single_group["torque_enable"].as<bool>(TORQUE_ENABLE)) {
      robot_torque_enable(CMD_TYPE_GROUP, name, false);
    }
  }

  YAML::Node all_singles = mode_configs["singles"];
  for (
    YAML::const_iterator single_itr = all_singles.begin();
    single_itr != all_singles.end();
    single_itr++)
  {
    std::string single_name = single_itr->first.as<std::string>();
    YAML::Node single_joint = all_singles[single_name];
    const std::string operating_mode =
      single_joint["operating_mode"].as<std::string>(DEFAULT_OP_MODE);
    const std::string profile_type =
      single_joint["profile_type"].as<std::string>(DEFAULT_PROF_TYPE);
    int32_t profile_velocity = single_joint["profile_velocity"].as<int32_t>(DEFAULT_PROF_VEL);
    int32_t profile_acceleration =
      single_joint["profile_acceleration"].as<int32_t>(DEFAULT_PROF_ACC);
    robot_set_operating_modes(
      CMD_TYPE_SINGLE,
      single_name,
      operating_mode,
      profile_type,
      profile_velocity,
      profile_acceleration);
    if (!single_joint["torque_enable"].as<bool>(TORQUE_ENABLE)) {
      robot_torque_enable(CMD_TYPE_SINGLE, single_name, false);
    }
  }
}

void InterbotixRobotXS::robot_init_publishers()
{
  if (pub_states) {
    pub_joint_states = this->create_publisher<sensor_msgs::msg::JointState>(js_topic, 10);
  }
}

void InterbotixRobotXS::robot_init_subscribers()
{
  using namespace std::placeholders;
  sub_command_group = this->create_subscription<JointGroupCommand>(
    "commands/joint_group",
    10,
    std::bind(&InterbotixRobotXS::robot_sub_command_group, this, _1));
  sub_command_single = this->create_subscription<JointSingleCommand>(
    "commands/joint_single",
    10,
    std::bind(&InterbotixRobotXS::robot_sub_command_single, this, _1));
  sub_command_traj = this->create_subscription<JointTrajectoryCommand>(
    "commands/joint_trajectory",
    10,
    std::bind(&InterbotixRobotXS::robot_sub_command_traj, this, _1));
}

void InterbotixRobotXS::robot_init_services()
{
  using namespace std::placeholders;
  srv_torque_enable = this->create_service<TorqueEnable>(
    "torque_enable",
    std::bind(&InterbotixRobotXS::robot_srv_torque_enable, this, _1, _2, _3));
  srv_reboot_motors = this->create_service<Reboot>(
    "reboot_motors",
    std::bind(&InterbotixRobotXS::robot_srv_reboot_motors, this, _1, _2, _3));
  srv_get_robot_info = this->create_service<RobotInfo>(
    "get_robot_info",
    std::bind(&InterbotixRobotXS::robot_srv_get_robot_info, this, _1, _2, _3));
  srv_operating_modes = this->create_service<OperatingModes>(
    "set_operating_modes",
    std::bind(&InterbotixRobotXS::robot_srv_set_operating_modes, this, _1, _2, _3));
  srv_motor_gains = this->create_service<MotorGains>(
    "set_motor_pid_gains",
    std::bind(&InterbotixRobotXS::robot_srv_set_motor_pid_gains, this, _1, _2, _3));
  srv_set_registers = this->create_service<RegisterValues>(
    "set_motor_registers",
    std::bind(&InterbotixRobotXS::robot_srv_set_motor_registers, this, _1, _2, _3));
  srv_get_registers = this->create_service<RegisterValues>(
    "get_motor_registers",
    std::bind(&InterbotixRobotXS::robot_srv_get_motor_registers, this, _1, _2, _3));
}

void InterbotixRobotXS::robot_init_timers()
{
  execute_joint_traj = false;
  using namespace std::chrono_literals;
  if (timer_hz != 0) {
    // timer that updates the joint states with a period of 1/timer_hz
    std::chrono::nanoseconds update_rate_in_ms = std::chrono::nanoseconds(
      static_cast<int>(1.0 / (timer_hz) * 1000000000.0));
    tmr_joint_states = this->create_wall_timer(
      update_rate_in_ms,
      std::bind(
        &InterbotixRobotXS::robot_update_joint_states,
        this));
  }
}

void InterbotixRobotXS::robot_wait_for_joint_states()
{
  if (timer_hz == 0) {
    return;
  }
  rclcpp::Rate r(10);
  while (rclcpp::ok() && joint_states.name.size() == 0) {
    rclcpp::spin_some(this->get_node_base_interface());
    r.sleep();
  }
}

void InterbotixRobotXS::robot_sub_command_group(const JointGroupCommand::SharedPtr msg)
{
  robot_write_commands(msg->name, msg->cmd);
}

void InterbotixRobotXS::robot_sub_command_single(const JointSingleCommand::SharedPtr msg)
{
  robot_write_joint_command(msg->name, msg->cmd);
}

void InterbotixRobotXS::robot_sub_command_traj(const JointTrajectoryCommand::SharedPtr msg)
{
  using namespace std::chrono_literals;
  if (execute_joint_traj) {
    RCLCPP_WARN(LOGGER, "Trajectory rejected since joints are still moving.");
    return;
  }

  if (msg->traj.points.size() < 2) {
    RCLCPP_WARN(LOGGER, "Trajectory has fewer than 2 points. Aborting...");
    return;
  }

  std::vector<std::string> joint_names;
  if (msg->cmd_type == CMD_TYPE_GROUP) {
    joint_names = group_map[msg->name].joint_names;
  } else if (msg->cmd_type == CMD_TYPE_SINGLE) {
    joint_names.push_back(msg->name);
  }

  if (timer_hz != 0 && msg->traj.points[0].positions.size() == joint_names.size()) {
    for (size_t i{0}; i < joint_names.size(); i++) {
      float expected_state = msg->traj.points[0].positions.at(i);
      float actual_state = joint_states.position.at(js_index_map[joint_names.at(i)]);
      if (!(fabs(expected_state - actual_state) < 0.01)) {
        RCLCPP_WARN(
          LOGGER,
          "The %s joint is not at the correct initial state.",
          joint_names.at(i).c_str());
        RCLCPP_WARN(
          LOGGER,
          "Expected state: %.2f, Actual State: %.2f.",
          expected_state, actual_state);
      }
    }
  }
  joint_traj_cmd = msg;
  execute_joint_traj = true;

  // create timer that immediately triggers callback
  tmr_joint_traj = create_wall_timer(
    0s,
    std::bind(
      &InterbotixRobotXS::robot_execute_trajectory,
      this));
}

bool InterbotixRobotXS::robot_srv_torque_enable(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<TorqueEnable::Request> req,
  const std::shared_ptr<TorqueEnable::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type.c_str(), req->name)) {
    return false;
  }

  robot_torque_enable(req->cmd_type.c_str(), req->name, req->enable);
  return true;
}

bool InterbotixRobotXS::robot_srv_reboot_motors(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<Reboot::Request> req,
  const std::shared_ptr<Reboot::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type.c_str(), req->name)) {
    return false;
  }

  robot_reboot_motors(
    req->cmd_type.c_str(),
    req->name,
    req->enable,
    req->smart_reboot);
  return true;
}

bool InterbotixRobotXS::robot_srv_get_robot_info(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RobotInfo::Request> req,
  std::shared_ptr<RobotInfo::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type.c_str(), req->name)) {
    return false;
  }
  bool urdf_exists = false;
  urdf::Model model;
  urdf::JointConstSharedPtr ptr;

  // Parse the urdf model to get joint limit info
  std::string robot_name = this->get_namespace();
  std::string robot_description;
  this->get_parameter("robot_description", robot_description);
  if (!robot_description.empty()) {
    model.initString(robot_description);
    urdf_exists = true;
  }

  if (req->cmd_type == CMD_TYPE_GROUP) {
    res->joint_names = group_map[req->name].joint_names;
    res->profile_type = group_map[req->name].profile_type;
    res->mode = group_map[req->name].mode;
  } else if (req->cmd_type == CMD_TYPE_SINGLE) {
    res->joint_names.push_back(req->name);
    res->profile_type = motor_map[req->name].profile_type;
    res->mode = motor_map[req->name].mode;
  }

  res->num_joints = res->joint_names.size();

  for (auto & name : res->joint_names) {
    res->joint_ids.push_back(motor_map[name].motor_id);
    if (gripper_map.count(name) > 0) {
      res->joint_sleep_positions.push_back(robot_convert_angular_position_to_linear(name, 0));
      name = gripper_map[name].left_finger;
    } else {
      res->joint_sleep_positions.push_back(sleep_map[name]);
    }

    res->joint_state_indices.push_back(js_index_map[name]);
    if (urdf_exists) {
      ptr = model.getJoint(name);
      res->joint_lower_limits.push_back(ptr->limits->lower);
      res->joint_upper_limits.push_back(ptr->limits->upper);
      res->joint_velocity_limits.push_back(ptr->limits->velocity);
    }
  }
  if (req->name != "all") {
    res->name.push_back(req->name);
  } else {
    for (auto const &[group_name, _] : group_map) {
      res->name.push_back(group_name);
    }
  }
  return true;
}

bool InterbotixRobotXS::robot_srv_set_operating_modes(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<OperatingModes::Request> req,
  const std::shared_ptr<OperatingModes::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type.c_str(), req->name)) {
    return false;
  }

  robot_set_operating_modes(
    req->cmd_type.c_str(),
    req->name,
    req->mode,
    req->profile_type.c_str(),
    req->profile_velocity,
    req->profile_acceleration);
  return true;
}

bool InterbotixRobotXS::robot_srv_set_motor_pid_gains(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<MotorGains::Request> req,
  const std::shared_ptr<MotorGains::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type.c_str(), req->name)) {
    return false;
  }

  std::vector<int32_t> gains = {
    req->kp_pos,
    req->ki_pos,
    req->kd_pos,
    req->k1,
    req->k2,
    req->kp_vel,
    req->ki_vel};
  robot_set_motor_pid_gains(req->cmd_type.c_str(), req->name, gains);
  return true;
}

bool InterbotixRobotXS::robot_srv_set_motor_registers(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RegisterValues::Request> req,
  const std::shared_ptr<RegisterValues::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type.c_str(), req->name)) {
    return false;
  }

  robot_set_motor_registers(req->cmd_type.c_str(), req->name, req->reg, req->value);
  return true;
}

bool InterbotixRobotXS::robot_srv_get_motor_registers(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RegisterValues::Request> req,
  std::shared_ptr<RegisterValues::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type.c_str(), req->name)) {
    return false;
  }

  robot_get_motor_registers(req->cmd_type.c_str(), req->name, req->reg, res->values);
  return true;
}

void InterbotixRobotXS::robot_execute_trajectory()
{
  // get the current real time for this callback execution
  rclcpp::Time current_real = this->get_clock()->now();

  static size_t cntr = 1;
  RCLCPP_DEBUG(
    LOGGER,
    "Executing trajectory step %li/%li.",
    cntr, joint_traj_cmd->traj.points.size());

  // check if end of trajectory has been reached if done, reset counter, set execution status bool
  // to false, and cancel the trajectory execution timer. if not done, cancel the timer (pseudo
  // one-shot), and start another.
  if (cntr == joint_traj_cmd->traj.points.size()) {
    if (!tmr_joint_traj->is_canceled()) {
      tmr_joint_traj->cancel();
    }
    execute_joint_traj = false;
    cntr = 1;
    RCLCPP_DEBUG(LOGGER, "Reached end of trajectory.");
    return;
  } else {
    // cancel trajectory timer
    if (!tmr_joint_traj->is_canceled()) {
      tmr_joint_traj->cancel();
    }

    // get the length of time the timer should be if perfect
    rclcpp::Duration period = std::chrono::nanoseconds(
      joint_traj_cmd->traj.points[cntr].time_from_start.nanosec -
      joint_traj_cmd->traj.points[cntr - 1].time_from_start.nanosec);

    // create new timer with the actual length of time it should execute
    //  (period - (now - start_of_callback))
    tmr_joint_traj = this->create_wall_timer(
      std::chrono::nanoseconds(
        period.nanoseconds() - (
          this->get_clock()->now().nanoseconds() - current_real.nanoseconds())),
      std::bind(
        &InterbotixRobotXS::robot_execute_trajectory,
        this));
  }

  // get the mode
  const std::string mode = group_map[joint_traj_cmd->name].mode;

  // write commands to the motors depending on cmd_type and mode
  if (joint_traj_cmd->cmd_type == CMD_TYPE_GROUP) {
    if (
      (mode == MODE_POSITION) ||
      (mode == MODE_EXT_POSITION) ||
      (mode == MODE_CURRENT_BASED_POSITION) ||
      (mode == MODE_LINEAR_POSITION))
    {
      std::vector<float> commands(
        joint_traj_cmd->traj.points[cntr].positions.begin(),
        joint_traj_cmd->traj.points[cntr].positions.end());
      robot_write_commands(joint_traj_cmd->name, commands);
    } else if (group_map[joint_traj_cmd->name].mode == MODE_VELOCITY) {
      std::vector<float> commands(
        joint_traj_cmd->traj.points[cntr].velocities.begin(),
        joint_traj_cmd->traj.points[cntr].velocities.end());
      robot_write_commands(joint_traj_cmd->name, commands);
    } else if ( // NOLINT https://github.com/ament/ament_lint/issues/158
      (group_map[joint_traj_cmd->name].mode == MODE_PWM) ||
      (group_map[joint_traj_cmd->name].mode == MODE_CURRENT))
    {
      std::vector<float> commands(
        joint_traj_cmd->traj.points[cntr].effort.begin(),
        joint_traj_cmd->traj.points[cntr].effort.end());
      robot_write_commands(joint_traj_cmd->name, commands);
    }
  } else if (joint_traj_cmd->cmd_type == CMD_TYPE_SINGLE) {
    if (
      (mode == MODE_POSITION) ||
      (mode == MODE_EXT_POSITION) ||
      (mode == MODE_CURRENT_BASED_POSITION) ||
      (mode == MODE_LINEAR_POSITION))
    {
      robot_write_joint_command(
        joint_traj_cmd->name,
        joint_traj_cmd->traj.points[cntr].positions.at(0));
    } else if (motor_map[joint_traj_cmd->name].mode == MODE_VELOCITY) {
      robot_write_joint_command(
        joint_traj_cmd->name,
        joint_traj_cmd->traj.points[cntr].velocities.at(0));
    } else if ( // NOLINT https://github.com/ament/ament_lint/issues/158
      (motor_map[joint_traj_cmd->name].mode == MODE_PWM) ||
      (motor_map[joint_traj_cmd->name].mode == MODE_CURRENT))
    {
      robot_write_joint_command(
        joint_traj_cmd->name,
        joint_traj_cmd->traj.points[cntr].effort.at(0));
    }
  }
  cntr++;
}

void InterbotixRobotXS::robot_update_joint_states()
{
  const char * log;

  sensor_msgs::msg::JointState joint_state_msg;

  std::vector<int32_t> get_current(all_ptr->joint_num, 0);
  std::vector<int32_t> get_velocity(all_ptr->joint_num, 0);
  std::vector<int32_t> get_position(all_ptr->joint_num, 0);
  joint_state_msg.name = all_ptr->joint_names;

  if (dxl_wb.getProtocolVersion() == 2.0f) {
    // Checks if data can be sent properly
    if (!dxl_wb.syncRead(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        all_ptr->joint_ids.data(),
        all_ptr->joint_num,
        &log))
    {
      RCLCPP_ERROR(LOGGER, "%s", log);
    }

    // Gets present current of all servos
    if (!dxl_wb.getSyncReadData(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        all_ptr->joint_ids.data(),
        all_ptr->joint_num,
        control_items["Present_Current"]->address,
        control_items["Present_Current"]->data_length,
        get_current.data(),
        &log))
    {
      RCLCPP_ERROR(LOGGER, "%s", log);
    }

    // Gets present velocity of all servos
    if (!dxl_wb.getSyncReadData(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        all_ptr->joint_ids.data(),
        all_ptr->joint_num,
        control_items["Present_Velocity"]->address,
        control_items["Present_Velocity"]->data_length,
        get_velocity.data(),
        &log))
    {
      RCLCPP_ERROR(LOGGER, "%s", log);
    }

    // Gets present position of all servos
    if (!dxl_wb.getSyncReadData(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        all_ptr->joint_ids.data(),
        all_ptr->joint_num,
        control_items["Present_Position"]->address,
        control_items["Present_Position"]->data_length,
        get_position.data(),
        &log))
    {
      RCLCPP_ERROR(LOGGER, "%s", log);
    }

    uint8_t index = 0;
    for (auto const & id : all_ptr->joint_ids) {
      float position = 0;
      float velocity = 0;
      float effort = 0;

      if (strcmp(dxl_wb.getModelName(id), "XL-320") == 0) {
        effort = dxl_wb.convertValue2Load(get_current.at(index));
      } else {
        effort = dxl_wb.convertValue2Current(get_current.at(index));
      }
      velocity = dxl_wb.convertValue2Velocity(id, get_velocity.at(index));
      position = dxl_wb.convertValue2Radian(id, get_position.at(index));
      joint_state_msg.effort.push_back(effort);
      joint_state_msg.velocity.push_back(velocity);
      joint_state_msg.position.push_back(position);
      index++;
    }
  } else if (dxl_wb.getProtocolVersion() == 1.0f) {
    uint16_t length_of_data = control_items["Present_Position"]->data_length +
      control_items["Present_Velocity"]->data_length +
      control_items["Present_Current"]->data_length;
    std::vector<uint32_t> get_all_data(length_of_data, 0);

    for (auto const & id : all_ptr->joint_ids) {
      if (!dxl_wb.readRegister(
          id,
          control_items["Present_Position"]->address,
          length_of_data,
          get_all_data.data(),
          &log))
      {
        RCLCPP_ERROR(LOGGER, "%s", log);
      }

      int16_t effort_raw = DXL_MAKEWORD(get_all_data.at(4), get_all_data.at(5));
      int32_t velocity_raw = DXL_MAKEWORD(get_all_data.at(2), get_all_data.at(3));
      int32_t position_raw = DXL_MAKEWORD(get_all_data.at(0), get_all_data.at(1));

      // Convert raw register values to the metric system
      float effort = dxl_wb.convertValue2Load(effort_raw);
      float velocity = dxl_wb.convertValue2Velocity(id, velocity_raw);
      float position = dxl_wb.convertValue2Radian(id, position_raw);

      joint_state_msg.effort.push_back(effort);
      joint_state_msg.velocity.push_back(velocity);
      joint_state_msg.position.push_back(position);
    }
  }

  for (auto const & name : gripper_order) {
    joint_state_msg.name.push_back(gripper_map[name].left_finger.c_str());
    joint_state_msg.name.push_back(gripper_map[name].right_finger.c_str());
    float pos = robot_convert_angular_position_to_linear(
      name, joint_state_msg.position.at(gripper_map[name].js_index));
    joint_state_msg.position.push_back(pos);
    joint_state_msg.position.push_back(-pos);
    joint_state_msg.velocity.push_back(0);
    joint_state_msg.velocity.push_back(0);
    joint_state_msg.effort.push_back(0);
    joint_state_msg.effort.push_back(0);
  }

  // Publish the message to the joint_states topic
  joint_state_msg.header.stamp = this->get_clock()->now();
  joint_states = joint_state_msg;
  if (pub_states) {
    pub_joint_states->publish(joint_state_msg);
  }
}

bool InterbotixRobotXS::robot_srv_validate(
  const std::string & cmd_type,
  const std::string & name)
{
  if (cmd_type == CMD_TYPE_GROUP) {
    if (group_map.count(name) > 0) {
      // if group name is valid, return true
      return true;
    } else {
      // otherwise error and return false
      RCLCPP_ERROR(
        LOGGER,
        "Group '%s' does not exist. Was it added to the motor config file?",
        name.c_str());
      return false;
    }
  } else if (cmd_type == CMD_TYPE_SINGLE) {
    if (motor_map.count(name) > 0) {
      // if joint name is valid, return true
      return true;
    } else {
      // otherwise error and return false
      RCLCPP_ERROR(
        LOGGER,
        "Joint '%s' does not exist. Was it added to the motor config file?",
        name.c_str());
      return false;
    }
  } else {
    // if command type is invalid, error and return false
    RCLCPP_ERROR(
      LOGGER,
      "cmd_type was '%s'. Choices are 'group' or 'single'.",
      cmd_type.c_str());
    return false;
  }
}

}  // namespace interbotix_xs
