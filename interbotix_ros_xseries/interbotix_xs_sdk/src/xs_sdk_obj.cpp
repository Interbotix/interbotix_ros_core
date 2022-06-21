#include "interbotix_xs_sdk/xs_sdk_obj.h"

/// @brief Constructor for the InterbotixRobotXS
/// @param node_handle - ROS NodeHandle
InterbotixRobotXS::InterbotixRobotXS(ros::NodeHandle *node_handle, bool &success)
    : node(*node_handle)
{
  if (!robot_get_motor_configs())
  {
    success = false;
    return;
  }

  if (!robot_init_port())
  {
    success = false;
    return;
  }

  if (!robot_ping_motors())
  {
    success = false;
    ROS_FATAL("[xs_sdk] Failed to find all motors specified in the motor_config file after three attempts. Shutting down...");
    return;
  }

  if (!robot_load_motor_configs())
  {
    success = false;
    ROS_FATAL("[xs_sdk] Failed to write configurations to all motors. Shutting down...");
    return;
  }

  robot_init_controlItems();
  robot_init_SDK_handlers();
  robot_init_operating_modes();
  robot_init_publishers();
  robot_init_subscribers();
  robot_init_services();
  robot_init_timers();
  robot_wait_for_joint_states();
  ROS_INFO("[xs_sdk] Interbotix 'xs_sdk' node is up!");
}

/// @brief Destructor for the InterbotixRobotXS
InterbotixRobotXS::~InterbotixRobotXS(){}

/// @brief Set the operating mode for a specific group of motors or a single motor
/// @param cmd_type - set to 'group' if changing the operating mode for a group of motors or 'single' if changing the operating mode for a single motor
/// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
/// @param mode - desired operating mode (either 'position', 'linear_position', 'ext_position', 'velocity', 'pwm', 'current', or 'current_based_position')
/// @param profile_type - set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based Profile (modifies Bit 2 of the 'Drive_Mode' register)
/// @param profile_velocity - passthrough to the Profile_Velocity register on the motor
/// @param profile_acceleration - passthrough to the Profile_Acceleration register on the motor
void InterbotixRobotXS::robot_set_operating_modes(std::string const& cmd_type, std::string const& name, std::string const& mode, std::string const& profile_type, int32_t profile_velocity, int32_t profile_acceleration)
{
  if (cmd_type == "group" && group_map.count(name) > 0)
  {
    for (auto const& joint_name:group_map[name].joint_names)
      robot_set_joint_operating_mode(joint_name, mode, profile_type, profile_velocity, profile_acceleration);
    group_map[name].mode = mode;
    group_map[name].profile_type = profile_type;
    ROS_INFO("[xs_sdk] The operating mode for the '%s' group was changed to %s.", name.c_str(), mode.c_str());
  }
  else if (cmd_type == "single" && motor_map.count(name) > 0)
  {
    robot_set_joint_operating_mode(name, mode, profile_type, profile_velocity, profile_acceleration);
    ROS_INFO("[xs_sdk] The operating mode for the '%s' joint was changed to %s.", name.c_str(), mode.c_str());
  }
  else if (cmd_type == "group" && group_map.count(name) == 0 || cmd_type == "single" && motor_map.count(name) == 0)
    ROS_WARN("[xs_sdk] The '%s' joint/group does not exist. Was it added to the motor config file?", name.c_str());
  else
    ROS_ERROR("[xs_sdk] Invalid command for argument 'cmd_type' while setting operating mode.");
}

/// @brief Helper function used to set the operating mode for a single motor
/// @param name - desired motor name
/// @param mode - desired operating mode (either 'position', 'linear_position', 'ext_position', 'velocity', 'pwm', 'current', or 'current_based_position')
/// @param profile_type - set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based Profile (modifies Bit 2 of the 'Drive_Mode' register)
/// @param profile_velocity - passthrough to the Profile_Velocity register on the motor
/// @param profile_acceleration - passthrough to the Profile_Acceleration register on the motor
void InterbotixRobotXS::robot_set_joint_operating_mode(std::string const& name, std::string const& mode, std::string const& profile_type, int32_t profile_velocity, int32_t profile_acceleration)
{
  for (auto const& joint_name:sister_map[name])
  {
    dxl_wb.torque(motor_map[joint_name].motor_id, false);
    ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, torqued off.", motor_map[joint_name].motor_id);
  }

  for (auto const& motor_name:shadow_map[name])
  {
    int32_t drive_mode;
    dxl_wb.itemRead(motor_map[motor_name].motor_id, "Drive_Mode", &drive_mode);
    ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, read Drive Mode %d.", motor_map[motor_name].motor_id, drive_mode);

    if (drive_mode <= 1 && profile_type == "time")
    {
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Drive_Mode", drive_mode + 4);
      ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, write Drive Mode %d.", motor_map[motor_name].motor_id, drive_mode + 4);
    }
    else if (drive_mode >= 4 && profile_type == "velocity")
    {
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Drive_Mode", drive_mode - 4);
      ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, write Drive Mode %d.", motor_map[motor_name].motor_id, drive_mode - 4);
    }

    if (mode == "position" || mode == "linear_position")
    {
      dxl_wb.setPositionControlMode(motor_map[motor_name].motor_id);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Velocity", profile_velocity);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Acceleration", profile_acceleration);
      ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, set poscontrolmode, pv=%i, pa=%i.", motor_map[motor_name].motor_id, profile_velocity, profile_acceleration);
    }
    else if (mode == "ext_position")
    {
      dxl_wb.setExtendedPositionControlMode(motor_map[motor_name].motor_id);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Velocity", profile_velocity);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Acceleration", profile_acceleration);
      ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, set extposcontrolmode, pv=%i, pa=%i.", motor_map[motor_name].motor_id, profile_velocity, profile_acceleration);

    }
    else if (mode == "velocity")
    {
      dxl_wb.setVelocityControlMode(motor_map[motor_name].motor_id);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Acceleration", profile_acceleration);
      ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, set velcontrolmode, pa=%i.", motor_map[motor_name].motor_id, profile_acceleration);
    }
    else if (mode == "pwm")
    {
      dxl_wb.setPWMControlMode(motor_map[motor_name].motor_id);
      ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, set pwmcontrolmode.", motor_map[motor_name].motor_id);
    }

    else if (mode == "current")
    {
      dxl_wb.setCurrentControlMode(motor_map[motor_name].motor_id);
      ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, set currentcontrolmode", motor_map[motor_name].motor_id);
    }
    else if (mode == "current_based_position")
    {
      dxl_wb.setCurrentBasedPositionControlMode(motor_map[motor_name].motor_id);
      ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, set currentcontrolmode", motor_map[motor_name].motor_id);
    }
    else
    {
      ROS_ERROR("[xs_sdk] Invalid command for argument 'mode' while setting the operating mode for the %s motor.", motor_name.c_str());
      continue;
    }
    motor_map[motor_name].mode = mode;
    motor_map[motor_name].profile_type = profile_type;
  }

  for (auto const& joint_name:sister_map[name])
  {
    dxl_wb.torque(motor_map[joint_name].motor_id, true);
    ROS_DEBUG("[xs_sdk::robot_set_joint_operating_mode] ID: %d, torqued on.", motor_map[joint_name].motor_id);
  }

}

/// @brief Torque On/Off a specific group of motors or a single motor
/// @param cmd_type - set to 'group' if torquing off a group of motors or 'single' if torquing off a single motor
/// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
/// @param enable - set to True to torque on or False to torque off
void InterbotixRobotXS::robot_torque_enable(std::string const& cmd_type, std::string const& name, bool const& enable)
{
  if (cmd_type == "group" && group_map.count(name) > 0)
  {
    for (auto const& joint_name:group_map[name].joint_names)
      dxl_wb.torque(motor_map[joint_name].motor_id, enable);
    if (enable) ROS_INFO("[xs_sdk] The '%s' group was torqued on.", name.c_str());
    else ROS_INFO("[xs_sdk] The '%s' group was torqued off.", name.c_str());
  }
  else if (cmd_type == "single" && motor_map.count(name) > 0)
  {
    dxl_wb.torque(motor_map[name].motor_id, enable);
    if (enable) ROS_INFO("[xs_sdk] The '%s' joint was torqued on.", name.c_str());
    else ROS_INFO("[xs_sdk] The '%s' joint was torqued off.", name.c_str());
  }
  else if (cmd_type == "group" && group_map.count(name) == 0 || cmd_type == "single" && motor_map.count(name) == 0)
    ROS_WARN("[xs_sdk] The '%s' joint/group does not exist. Was it added to the motor config file?", name.c_str());
  else
    ROS_ERROR("[xs_sdk] Invalid command for argument 'cmd_type' while torquing joints.");
}

/// @brief Reboot a specific group of motors or a single motor
/// @param cmd_type - set to 'group' if rebooting a group of motors or 'single' if rebooting a single motor
/// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
/// @param torque_enable - set to True to torque on or False to torque off after rebooting
/// @param smart_reboot - set to True to only reboot motor(s) in a specified group that have gone into an error state
void InterbotixRobotXS::robot_reboot_motors(std::string const& cmd_type, std::string const& name, bool const& enable, bool const& smart_reboot)
{
  std::vector<std::string> joints_to_torque;
  if (cmd_type == "group" && group_map.count(name) > 0)
  {
    for (auto const& joint_name:group_map[name].joint_names)
    {
      if (smart_reboot)
      {
        int32_t value = 0;
        const char *log;
        bool success = dxl_wb.itemRead(motor_map[joint_name].motor_id, "Hardware_Error_Status", &value, &log);
        if (success && value == 0)
          continue;
      }
      dxl_wb.reboot(motor_map[joint_name].motor_id);
      ROS_INFO("[xs_sdk] The '%s' joint was rebooted.", joint_name.c_str());
      if (enable) joints_to_torque.push_back(joint_name);
    }
    if (!smart_reboot)
      ROS_INFO("[xs_sdk] The '%s' group was rebooted.", name.c_str());
  }
  else if (cmd_type == "single" && motor_map.count(name) > 0)
  {
    dxl_wb.reboot(motor_map[name].motor_id);
    ROS_INFO("[xs_sdk] The '%s' joint was rebooted.", name.c_str());
    if (enable) joints_to_torque.push_back(name);
  }
  else if (cmd_type == "group" && group_map.count(name) == 0 || cmd_type == "single" && motor_map.count(name) == 0)
    ROS_WARN("[xs_sdk] The '%s' joint/group does not exist. Was it added to the motor config file?", name.c_str());
  else
    ROS_ERROR("[xs_sdk] Invalid command for argument 'cmd_type' while rebooting motors.");

  for (auto const& joint_name:joints_to_torque)
  {
    for (auto const& name:sister_map[joint_name])
      robot_torque_enable("single", name, true);
  }
}

/// @brief Command a desired group of motors with the specified commands
/// @param name - desired motor group name
/// @param commands - vector of commands (order matches the order specified in the 'groups' section in the motor config file)
/// @details - commands are processed differently based on the operating mode specified for the motor group
void InterbotixRobotXS::robot_write_commands(std::string const& name, std::vector<float> commands)
{
  if (commands.size() != group_map[name].joint_num)
  {
    ROS_ERROR("[xs_sdk] Number of commands (%ld) does not match the number of joints in group '%s' (%d). Will not execute.", commands.size(), name.c_str(), group_map[name].joint_num);
    return;
  }
  std::string mode = group_map[name].mode;
  int32_t dynamixel_commands[commands.size()];

  if (mode == "position" || mode == "ext_position" || mode == "current_based_position" || mode == "linear_position")
  {
    for (size_t i{0}; i < commands.size(); i++)
    {
      if (mode == "linear_position")
        commands.at(i) = robot_convert_linear_position_to_radian(group_map[name].joint_names.at(i), commands.at(i));
      dynamixel_commands[i] = dxl_wb.convertRadian2Value(group_map[name].joint_ids.at(i), commands.at(i));
      ROS_DEBUG("[xs_sdk::robot_write_commands] ID: %d, writing %s command %d.", group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, group_map[name].joint_ids.data(), group_map[name].joint_num, dynamixel_commands, 1);
  }
  else if (mode == "velocity")
  {
    for (size_t i{0}; i < commands.size(); i++)
    {
      dynamixel_commands[i] = dxl_wb.convertVelocity2Value(group_map[name].joint_ids.at(i), commands.at(i));
      ROS_DEBUG("[xs_sdk::robot_write_commands] ID: %d, writing %s command %d.", group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, group_map[name].joint_ids.data(), group_map[name].joint_num, dynamixel_commands, 1);
  }
  else if (mode == "current")
  {
    for (size_t i{0}; i < commands.size(); i++)
    {
      dynamixel_commands[i] = dxl_wb.convertCurrent2Value(commands.at(i));
      ROS_DEBUG("[xs_sdk::robot_write_commands] ID: %d, writing %s command %d.", group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, group_map[name].joint_ids.data(), group_map[name].joint_num, dynamixel_commands, 1);
  }
  else if (mode == "pwm")
  {
    for (size_t i{0}; i < commands.size(); i++)
    {
      dynamixel_commands[i] = int32_t(commands.at(i));
      ROS_DEBUG("[xs_sdk::robot_write_commands] ID: %d, writing %s command %d.", group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_PWM, group_map[name].joint_ids.data(), group_map[name].joint_num, dynamixel_commands, 1);
  }
  else
    ROS_ERROR("[xs_sdk] Invalid command for argument 'mode' while commanding joint group.");
}

/// @brief Command a desired motor with the specified command
/// @param name - desired motor name
/// @param command - motor command
/// @details - the command is processed differently based on the operating mode specified for the motor
void InterbotixRobotXS::robot_write_joint_command(std::string const& name, float command)
{
  std::string mode = motor_map[name].mode;
  if (mode == "position" || mode == "ext_position" || mode == "current_based_position" || mode == "linear_position")
  {
    if (mode == "linear_position")
      command = robot_convert_linear_position_to_radian(name, command);
    ROS_DEBUG("[xs_sdk::robot_write_joint_command] ID: %d, writing %s command %f.", motor_map[name].motor_id, mode.c_str(), command);
    dxl_wb.goalPosition(motor_map[name].motor_id, command);
  }
  else if (mode == "velocity")
  {
    ROS_DEBUG("[xs_sdk::robot_write_joint_command] ID: %d, writing %s command %f.", motor_map[name].motor_id, mode.c_str(), command);
    dxl_wb.goalVelocity(motor_map[name].motor_id, command);
  }
  else if (mode == "current")
  {
    ROS_DEBUG("[xs_sdk::robot_write_joint_command] ID: %d, writing %s command %f.", motor_map[name].motor_id, mode.c_str(), command);
    dxl_wb.itemWrite(motor_map[name].motor_id, "Goal_Current", dxl_wb.convertCurrent2Value(command));
  }
  else if (mode == "pwm")
  {
    ROS_DEBUG("[xs_sdk::robot_write_joint_command] ID: %d, writing %s command %f.", motor_map[name].motor_id, mode.c_str(), command);
    dxl_wb.itemWrite(motor_map[name].motor_id, "Goal_PWM", int32_t(command));
  }
  else
    ROS_ERROR("[xs_sdk] Invalid command for argument 'mode' while commanding joint.");
}

/// @brief Set motor firmware PID gains
/// @param cmd_type - set to 'group' if changing the PID gains for a group of motors or 'single' if changing the PID gains for a single motor
/// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
/// @param gains - vector containing the desired PID gains - order is as shown in the function
void InterbotixRobotXS::robot_set_motor_pid_gains(std::string const& cmd_type, std::string const& name, std::vector<int32_t> const& gains)
{
  std::vector<std::string> names;
  if (cmd_type == "group")
    names = group_map[name].joint_names;
  else if (cmd_type == "single")
    names.push_back(name);

  for (auto const& name : names)
  {
    uint8_t id = motor_map[name].motor_id;
    ROS_DEBUG("[xs_sdk::robot_set_motor_pid_gains] ID: %d, writing gains:", motor_map[name].motor_id);
    ROS_DEBUG("[xs_sdk::robot_set_motor_pid_gains]         Pos_P: %i", gains.at(0));
    ROS_DEBUG("[xs_sdk::robot_set_motor_pid_gains]         Pos_I: %i", gains.at(1));
    ROS_DEBUG("[xs_sdk::robot_set_motor_pid_gains]         Pos_D: %i", gains.at(2));
    ROS_DEBUG("[xs_sdk::robot_set_motor_pid_gains]         FF_1: %i", gains.at(3));
    ROS_DEBUG("[xs_sdk::robot_set_motor_pid_gains]         FF_2: %i", gains.at(4));
    ROS_DEBUG("[xs_sdk::robot_set_motor_pid_gains]         Vel_P: %i", gains.at(5));
    ROS_DEBUG("[xs_sdk::robot_set_motor_pid_gains]         Vel_I: %i", gains.at(6));
    dxl_wb.itemWrite(id, "Position_P_Gain", gains.at(0));
    dxl_wb.itemWrite(id, "Position_I_Gain", gains.at(1));
    dxl_wb.itemWrite(id, "Position_D_Gain", gains.at(2));
    dxl_wb.itemWrite(id, "Feedforward_1st_Gain", gains.at(3));
    dxl_wb.itemWrite(id, "Feedforward_2nd_Gain", gains.at(4));
    dxl_wb.itemWrite(id, "Velocity_P_Gain", gains.at(5));
    dxl_wb.itemWrite(id, "Velocity_I_Gain", gains.at(6));
  }
}

/// @brief Set a register value to multiple motors
/// @param cmd_type - set to 'group' if setting register values for a group of motors or 'single' if setting a single register value
/// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
/// @param value - desired register value
void InterbotixRobotXS::robot_set_motor_registers(std::string const& cmd_type, std::string const& name, std::string const& reg, int32_t const& value)
{
  std::vector<std::string> names;
  if (cmd_type == "group")
    names = group_map[name].joint_names;
  else if (cmd_type == "single")
    names.push_back(name);

  for (auto const& name : names)
  {
    ROS_DEBUG("[xs_sdk::robot_set_motor_registers] ID: %d, writing reg: %s, value: %d.", motor_map[name].motor_id, reg.c_str(), value);
    dxl_wb.itemWrite(motor_map[name].motor_id, reg.c_str(), value);
  }
}

/// @brief Get a register value from multiple motors
/// @param cmd_type - set to 'group' if getting register values from a group of motors or 'single' if getting a single register value
/// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
/// @param values [out] - vector of register values
void InterbotixRobotXS::robot_get_motor_registers(std::string const& cmd_type, std::string const& name, std::string const& reg, std::vector<int32_t> &values)
{
  std::vector<std::string> names;
  if (cmd_type == "group")
    names = group_map[name].joint_names;
  else if (cmd_type == "single")
    names.push_back(name);

  const ControlItem *goal_reg = dxl_wb.getItemInfo(motor_map[names.front()].motor_id, reg.c_str());
  if (goal_reg == NULL)
  {
    ROS_ERROR("[xs_sdk] Could not get '%s' Item Info. Did you spell the register name correctly?", reg.c_str());
    return;
  }

  for (auto const& name : names)
  {
    int32_t value = 0;
    const char *log;
    if (!dxl_wb.itemRead(motor_map[name].motor_id, reg.c_str(), &value, &log))
    {
      ROS_ERROR("[xs_sdk] %s", log);
      return;
    }
    else
    {
      ROS_DEBUG("[xs_sdk::robot_get_motor_registers] ID: %d, reading reg: %s, value: %d.", motor_map[name].motor_id, reg.c_str(), value);
    }
    if (goal_reg->data_length == 1)
      values.push_back((uint8_t)value);
    else if (goal_reg->data_length == 2)
      values.push_back((int16_t)value);
    else
      values.push_back(value);
  }
}

/// @brief Get states for a group of joints
/// @param name - desired joint group name
/// @param positions [out] - vector of current joint positions [rad]
/// @param velocities [out] - vector of current joint velocities [rad/s]
/// @param effort [out] - vector of current joint effort [mA]
void InterbotixRobotXS::robot_get_joint_states(std::string const& name, std::vector<float> *positions, std::vector<float> *velocities, std::vector<float> *effort)
{
  for (auto const& joint_name : group_map[name].joint_names)
  {
    if (positions) positions->push_back(joint_states.position.at(js_index_map[joint_name]));
    if (velocities) velocities->push_back(joint_states.velocity.at(js_index_map[joint_name]));
    if (effort) effort->push_back(joint_states.effort.at(js_index_map[joint_name]));
    ROS_DEBUG("[xs_sdk::robot_get_joint_states] ID: %ld, got joint state.", js_index_map[joint_name]);
  }
}

/// @brief Get states for a single joint
/// @param name - desired joint name
/// @param position [out] - current joint position [rad]
/// @param velocity [out] - current joint velocity [rad/s]
/// @param effort [out] - current joint effort [mA]
void InterbotixRobotXS::robot_get_joint_state(std::string const& name, float *position, float *velocity, float *effort)
{
  if (position) *position = joint_states.position.at(js_index_map[name]);
  if (velocity) *velocity = joint_states.velocity.at(js_index_map[name]);
  if (effort) *effort = joint_states.effort.at(js_index_map[name]);
  ROS_DEBUG("[xs_sdk::robot_get_joint_state] ID: %ld, got joint state.", js_index_map[name]);
}

/// @brief Converts a desired linear distance between two gripper fingers into an angular position
/// @param name - name of the gripper servo to command
/// @param linear_position - desired distance [m] between the two gripper fingers
/// @param <float> [out] - angular position [rad] that achieves the desired linear distance
float InterbotixRobotXS::robot_convert_linear_position_to_radian(std::string const& name, float const& linear_position)
{
  float half_dist = linear_position / 2.0;
  float arm_length = gripper_map[name].arm_length;
  float horn_radius = gripper_map[name].horn_radius;
  float result = 3.14159/2.0 - acos((pow(horn_radius, 2) + pow(half_dist,2) - pow(arm_length, 2)) / (2 * horn_radius * half_dist));
  return result;
}

/// @brief Converts a specified angular position into the linear distance from one gripper finger to the center of the gripper servo horn
/// @param name - name of the gripper sevo to command
/// @param angular_position - desired gripper angular position [rad]
/// @param <float> [out] - linear position [m] from a gripper finger to the center of the gripper servo horn
float InterbotixRobotXS::robot_convert_angular_position_to_linear(std::string const& name, float const& angular_position)
{
  float arm_length = gripper_map[name].arm_length;
  float horn_radius = gripper_map[name].horn_radius;
  float a1 = horn_radius * sin(angular_position);
  float c = sqrt(pow(horn_radius,2) - pow(a1,2));
  float a2 = sqrt(pow(arm_length,2) - pow(c,2));
  return a1 + a2;
}

/// @brief Loads a robot-specific 'motor_configs' yaml file and populates class variables with its contents
/// @param <bool> [out] - True if the motor configs were successfully retrieved; False otherwise
bool InterbotixRobotXS::robot_get_motor_configs(void)
{
  std::string motor_configs_file, mode_configs_file;
  ros::param::get("~motor_configs", motor_configs_file);
  try
  {
    motor_configs = YAML::LoadFile(motor_configs_file.c_str());
  }
  catch (YAML::BadFile &error)
  {
    ROS_FATAL("[xs_sdk] Motor Config file was not found or has a bad format. Shutting down...");
    ROS_FATAL("[xs_sdk] YAML Error: '%s'", error.what());
    return false;
  }
  if (motor_configs.IsNull())
  {
    ROS_FATAL("[xs_sdk] Motor Config file was not found. Shutting down...");
    return false;
  }

  ros::param::get("~mode_configs", mode_configs_file);
  try
  {
    mode_configs = YAML::LoadFile(mode_configs_file.c_str());
    ROS_INFO("[xs_sdk] Loaded mode configs from '%s'.", mode_configs_file.c_str());
  }
  catch (YAML::BadFile &error)
  {
    ROS_ERROR("[xs_sdk] Motor Config file was not found or has a bad format. Shutting down...");
    ROS_ERROR("[xs_sdk] YAML Error: '%s'", error.what());
    return false;
  }
  if (mode_configs.IsNull())
    ROS_INFO("[xs_sdk] Mode Config file is empty.");

  port = motor_configs["port"].as<std::string>(PORT);
  if (mode_configs["port"])
    port = mode_configs["port"].as<std::string>(PORT);

  YAML::Node all_motors = motor_configs["motors"];
  for (YAML::const_iterator motor_itr = all_motors.begin(); motor_itr != all_motors.end(); motor_itr++)
  {
    std::string motor_name = motor_itr->first.as<std::string>();
    YAML::Node single_motor = all_motors[motor_name];
    uint8_t id = (uint8_t)single_motor["ID"].as<int32_t>();
    motor_map.insert({motor_name, {id, "position", "velocity"}});
    for (YAML::const_iterator info_itr = single_motor.begin(); info_itr != single_motor.end(); info_itr++)
    {
      std::string reg = info_itr->first.as<std::string>();
      if (reg != "ID" && reg != "Baud_Rate")
      {
        int32_t value = info_itr->second.as<int32_t>();
        MotorInfo motor_info = {id, reg, value};
        motor_info_vec.push_back(motor_info);
      }
    }
  }

  YAML::Node all_grippers = motor_configs["grippers"];
  for (YAML::const_iterator gripper_itr = all_grippers.begin(); gripper_itr != all_grippers.end(); gripper_itr++)
  {
    std::string gripper_name = gripper_itr->first.as<std::string>();
    YAML::Node single_gripper = all_grippers[gripper_name];
    Gripper gripper;
    gripper.horn_radius = single_gripper["horn_radius"].as<float>(0.014);
    gripper.arm_length = single_gripper["arm_length"].as<float>(0.024);
    gripper.left_finger = single_gripper["left_finger"].as<std::string>("left_finger");
    gripper.right_finger = single_gripper["right_finger"].as<std::string>("right_finger");
    gripper_map.insert({gripper_name, gripper});
  }

  YAML::Node joint_order = motor_configs["joint_order"];
  YAML::Node sleep_positions = motor_configs["sleep_positions"];
  JointGroup all_joints;
  all_joints.joint_num = (uint8_t) joint_order.size();
  all_joints.mode = "position";
  all_joints.profile_type = "velocity";
  for (size_t i{0}; i < joint_order.size(); i++)
  {
    std::string joint_name = joint_order[i].as<std::string>();
    all_joints.joint_names.push_back(joint_name);
    all_joints.joint_ids.push_back(motor_map[joint_name].motor_id);
    js_index_map.insert({joint_name, i});
    shadow_map.insert({joint_name, {joint_name}});
    sister_map.insert({joint_name, {joint_name}});
    sleep_map.insert({joint_name, sleep_positions[i].as<float>(0)});
    if (gripper_map.count(joint_name) > 0)
    {
      gripper_map[joint_name].js_index = i;
      gripper_order.push_back(joint_name);
    }
  }

  for (auto const& name : gripper_order)
  {
    js_index_map.insert({gripper_map[name].left_finger, js_index_map.size()});
    js_index_map.insert({gripper_map[name].right_finger, js_index_map.size()});
  }

  group_map.insert({"all", all_joints});
  all_ptr = &group_map["all"];

  YAML::Node all_shadows = motor_configs["shadows"];
  for (YAML::const_iterator master_itr = all_shadows.begin(); master_itr != all_shadows.end(); master_itr++)
  {
    std::string master_name = master_itr->first.as<std::string>();
    YAML::Node master = all_shadows[master_name];
    YAML::Node shadow_list = master["shadow_list"];
    for (size_t i{0}; i < shadow_list.size(); i++)
      shadow_map[master_name].push_back(shadow_list[i].as<std::string>());
  }

  YAML::Node all_sisters = motor_configs["sisters"];
  for (YAML::const_iterator sister_itr = all_sisters.begin(); sister_itr != all_sisters.end(); sister_itr++)
  {
    std::string sister_one = sister_itr->first.as<std::string>();
    std::string sister_two = sister_itr->second.as<std::string>();
    sister_map[sister_one].push_back(sister_two);
    sister_map[sister_two].push_back(sister_one);
  }

  YAML::Node all_groups = motor_configs["groups"];
  for (YAML::const_iterator group_itr = all_groups.begin(); group_itr != all_groups.end(); group_itr++)
  {
    std::string name = group_itr->first.as<std::string>();
    YAML::Node joint_list = all_groups[name];
    JointGroup group;
    group.joint_num = (uint8_t) joint_list.size();
    for (size_t i{0}; i < joint_list.size(); i++)
    {
      std::string joint_name = joint_list[i].as<std::string>();
      group.joint_names.push_back(joint_name);
      group.joint_ids.push_back(motor_map[joint_name].motor_id);
    }
    group_map.insert({name, group});
  }

  YAML::Node pub_configs = motor_configs["joint_state_publisher"];
  timer_hz = pub_configs["update_rate"].as<int>(100);
  pub_states = pub_configs["publish_states"].as<bool>(true);
  js_topic = pub_configs["topic_name"].as<std::string>("joint_states");

  ROS_INFO("[xs_sdk] Loaded motor configs from '%s'.", motor_configs_file.c_str());
  return true;
}

/// @brief Initializes the port to communicate with the Dynamixel servos
/// @param <bool> [out] - True if the port was successfully opened; False otherwise
bool InterbotixRobotXS::robot_init_port(void)
{
  if (!dxl_wb.init(port.c_str(), BAUDRATE))
  {
    ROS_FATAL("[xs_sdk] Failed to open port at %s. Shutting down...", port.c_str());
    return false;
  }
  return true;
}

/// @brief Pings all motors to make sure they can be found
bool InterbotixRobotXS::robot_ping_motors(void)
{
  bool found_all_motors = true;
  const char * log;
  for (size_t cntr_ping_motors=0; cntr_ping_motors<3; cntr_ping_motors++)
  {
    ROS_INFO(
      "[xs_sdk] Pinging all motors specified in the motor_config file. (Attempt %ld/3)",
      cntr_ping_motors+1);
    for (auto const& motor:motor_map)
    {
      if(!dxl_wb.ping(motor.second.motor_id, &log))
      {
        ROS_ERROR(
          "[xs_sdk]\tCan't find DYNAMIXEL ID: %2.d, Joint Name: '%s':\n\t\t  '%s'",
          motor.second.motor_id, motor.first.c_str(), log);
        found_all_motors = false;
      }
      else
        ROS_INFO(
          "[xs_sdk]\tFound DYNAMIXEL ID: %2.d, Model: '%s', Joint Name: '%s'.",
          motor.second.motor_id, dxl_wb.getModelName(motor.second.motor_id), motor.first.c_str());
      dxl_wb.torque(motor.second.motor_id, false);
    }
    if (found_all_motors)
      return found_all_motors;
  }
  return found_all_motors;
}

/// @brief Writes some 'startup' EEPROM register values to the Dynamixel servos
/// @param <bool> [out] - True if all register values were written successfully; False otherwise
bool InterbotixRobotXS::robot_load_motor_configs(void)
{
  ros::param::param<bool>("~load_configs", load_configs, true);
  if (load_configs)
  {
    ROS_INFO(
      "[xs_sdk] Writing startup register values to EEPROM. This only needs to be done once on a "
      "robot. Set the `~load_configs` parameter to false from now on.");
    for (auto const& motor_info:motor_info_vec)
    {
      if (!dxl_wb.itemWrite(motor_info.motor_id, motor_info.reg.c_str(), motor_info.value))
      {
        ROS_ERROR(
          "[xs_sdk] Failed to write value[%d] on items[%s] to [ID : %d]",
          motor_info.value, motor_info.reg.c_str(), motor_info.motor_id);
        return false;
      }
    }
  }
  else
    ROS_INFO("[xs_sdk] Skipping writing startup register values to EEPROM.");
  return true;
}

/// @brief Retrieves information about 'Goal_XXX' and 'Present_XXX' registers
/// @details - Info includes a register's name, address, and data length
void InterbotixRobotXS::robot_init_controlItems(void)
{
  uint8_t motor_id = motor_map.begin()->second.motor_id;

  const ControlItem *goal_position = dxl_wb.getItemInfo(motor_id, "Goal_Position");
  if (!goal_position)
    ROS_ERROR("[xs_sdk] Could not get 'Goal_Position' Item Info");

  const ControlItem *goal_velocity = dxl_wb.getItemInfo(motor_id, "Goal_Velocity");
  if (!goal_velocity) goal_velocity = dxl_wb.getItemInfo(motor_id, "Moving_Speed");
  if (!goal_velocity)
    ROS_ERROR("[xs_sdk] Could not get 'Goal_Velocity' or 'Moving_Speed' Item Info");

  const ControlItem *goal_current = NULL;
  for (auto const& motor:motor_map)
  {
    goal_current = dxl_wb.getItemInfo(motor.second.motor_id, "Goal_Current");
    if (goal_current)
      break;
  }
  if (!goal_current)
    ROS_INFO(
      "[xs_sdk] Could not get 'Goal_Current' Item Info. This message can be "
      "ignored if none of the robot's motors support current control.");

  const ControlItem *goal_pwm = dxl_wb.getItemInfo(motor_id, "Goal_PWM");
  if (!goal_pwm)
    ROS_ERROR("[xs_sdk] Could not get 'Goal_PWM' Item Info");

  const ControlItem *present_position = dxl_wb.getItemInfo(motor_id, "Present_Position");
  if (!present_position)
    ROS_ERROR("[xs_sdk] Could not get 'Present_Position' Item Info");

  const ControlItem *present_velocity = dxl_wb.getItemInfo(motor_id, "Present_Velocity");
  if (!present_velocity) present_velocity = dxl_wb.getItemInfo(motor_id, "Present_Speed");
  if (!present_velocity)
    ROS_ERROR("[xs_sdk] Could not get 'Present_Velocity' or 'Present_Speed' Item Info");

  const ControlItem *present_current = dxl_wb.getItemInfo(motor_id, "Present_Current");
  if (!present_current) present_current = dxl_wb.getItemInfo(motor_id, "Present_Load");
  if (!present_current)
    ROS_ERROR("[xs_sdk] Could not get 'Present_Current' or 'Present_Load' Item Info");

  control_items["Goal_Position"] = goal_position;
  control_items["Goal_Velocity"] = goal_velocity;
  control_items["Goal_Current"] = goal_current;
  control_items["Goal_PWM"] = goal_pwm;

  control_items["Present_Position"] = present_position;
  control_items["Present_Velocity"] = present_velocity;
  control_items["Present_Current"] = present_current;
}

  /// @brief Creates SyncWrite and SyncRead Handlers to write/read data to multiple motors simultaneously
void InterbotixRobotXS::robot_init_SDK_handlers(void)
{
  if (!dxl_wb.addSyncWriteHandler(control_items["Goal_Position"]->address, control_items["Goal_Position"]->data_length))
    ROS_ERROR("[xs_sdk] Failed to add SyncWriteHandler for Goal_Position.");

  if (!dxl_wb.addSyncWriteHandler(control_items["Goal_Velocity"]->address, control_items["Goal_Velocity"]->data_length))
    ROS_ERROR("[xs_sdk] Failed to add SyncWriteHandler for Goal_Velocity.");

  // only add a SyncWriteHandler for 'Goal_Current' if the register actually exists!
  if (control_items["Goal_Current"])
  {
    if (!dxl_wb.addSyncWriteHandler(control_items["Goal_Current"]->address, control_items["Goal_Current"]->data_length))
      ROS_ERROR("[xs_sdk] Failed to add SyncWriteHandler for Goal_Current.");
  }
  else
    ROS_INFO("[xs_sdk] SyncWriteHandler for Goal_Current not added as it's not supported.");

  if (!dxl_wb.addSyncWriteHandler(control_items["Goal_PWM"]->address, control_items["Goal_PWM"]->data_length))
    ROS_ERROR("[xs_sdk] Failed to add SyncWriteHandler for Goal_PWM.");

  if (dxl_wb.getProtocolVersion() == 2.0f)
  {
    uint16_t start_address = std::min(control_items["Present_Position"]->address, control_items["Present_Current"]->address);
    /*
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */
    // uint16_t read_length = control_items["Present_Position"]->data_length + control_items["Present_Velocity"]->data_length + control_items["Present_Current"]->data_length;
    uint16_t read_length = control_items["Present_Position"]->data_length + control_items["Present_Velocity"]->data_length + control_items["Present_Current"]->data_length+2;
    if (!dxl_wb.addSyncReadHandler(start_address, read_length))
      ROS_ERROR("[xs_sdk] Failed to add SyncReadHandler");
  }
}

/// @brief Loads a 'mode_configs' yaml file containing desired operating modes and sets up the motors accordingly
void InterbotixRobotXS::robot_init_operating_modes(void)
{
  YAML::Node all_shadows = motor_configs["shadows"];
  for (YAML::const_iterator master_itr = all_shadows.begin(); master_itr != all_shadows.end(); master_itr++)
  {
    std::string master_name = master_itr->first.as<std::string>();
    YAML::Node master = all_shadows[master_name];
    if (master["calibrate"].as<bool>(false))
    {
      int32_t master_position;
      dxl_wb.itemRead(motor_map[master_name].motor_id, "Present_Position", &master_position);
      for (auto const& shadow_name : shadow_map[master_name])
      {
        if (shadow_name == master_name) continue;
        dxl_wb.itemWrite(motor_map[shadow_name].motor_id, "Homing_Offset", 0);
        int32_t shadow_position, shadow_drive_mode;
        dxl_wb.itemRead(motor_map[shadow_name].motor_id, "Present_Position", &shadow_position);
        dxl_wb.itemRead(motor_map[shadow_name].motor_id, "Drive_Mode", &shadow_drive_mode);
        bool shadow_forward = (shadow_drive_mode % 2 == 0);
        int32_t homing_offset;
        if (shadow_forward)
          homing_offset = master_position - shadow_position;
        else
          homing_offset = shadow_position - master_position;
        dxl_wb.itemWrite(motor_map[shadow_name].motor_id, "Homing_Offset", homing_offset);
      }
    }
  }

  YAML::Node all_groups = mode_configs["groups"];
  for (YAML::const_iterator group_itr = all_groups.begin(); group_itr != all_groups.end(); group_itr++)
  {
    std::string name = group_itr->first.as<std::string>();
    YAML::Node single_group = all_groups[name];
    std::string operating_mode = single_group["operating_mode"].as<std::string>(OP_MODE);
    std::string profile_type = single_group["profile_type"].as<std::string>(PROFILE_TYPE);
    int32_t profile_velocity = single_group["profile_velocity"].as<int32_t>(PROFILE_VELOCITY);
    int32_t profile_acceleration = single_group["profile_acceleration"].as<int32_t>(PROFILE_ACCELERATION);
    robot_set_operating_modes("group", name, operating_mode, profile_type, profile_velocity, profile_acceleration);
    if (!single_group["torque_enable"].as<bool>(TORQUE_ENABLE))
      robot_torque_enable("group", name, false);
  }

  YAML::Node all_singles = mode_configs["singles"];
  for (YAML::const_iterator single_itr = all_singles.begin(); single_itr != all_singles.end(); single_itr++)
  {
    std::string single_name = single_itr->first.as<std::string>();
    YAML::Node single_joint = all_singles[single_name];
    std::string operating_mode = single_joint["operating_mode"].as<std::string>(OP_MODE);
    std::string profile_type = single_joint["profile_type"].as<std::string>(PROFILE_TYPE);
    int32_t profile_velocity = single_joint["profile_velocity"].as<int32_t>(PROFILE_VELOCITY);
    int32_t profile_acceleration = single_joint["profile_acceleration"].as<int32_t>(PROFILE_ACCELERATION);
    robot_set_operating_modes("single", single_name, operating_mode, profile_type, profile_velocity, profile_acceleration);
    if (!single_joint["torque_enable"].as<bool>(TORQUE_ENABLE))
      robot_torque_enable("single", single_name, false);
  }
}

/// @brief Initialize ROS Publishers
void InterbotixRobotXS::robot_init_publishers(void)
{
  if (pub_states)
    pub_joint_states = node.advertise<sensor_msgs::JointState>(js_topic, 1);
}

/// @brief Initialize ROS Subscribers
void InterbotixRobotXS::robot_init_subscribers(void)
{
  sub_command_group = node.subscribe("commands/joint_group", 5, &InterbotixRobotXS::robot_sub_command_group, this);
  sub_command_single = node.subscribe("commands/joint_single", 5, &InterbotixRobotXS::robot_sub_command_single, this);
  sub_command_traj = node.subscribe("commands/joint_trajectory", 5, &InterbotixRobotXS::robot_sub_command_traj, this);
}

/// @brief Initialize ROS Services
void InterbotixRobotXS::robot_init_services(void)
{
  srv_torque_enable = node.advertiseService("torque_enable", &InterbotixRobotXS::robot_srv_torque_enable, this);
  srv_reboot_motors = node.advertiseService("reboot_motors", &InterbotixRobotXS::robot_srv_reboot_motors, this);
  srv_get_robot_info = node.advertiseService("get_robot_info", &InterbotixRobotXS::robot_srv_get_robot_info, this);
  srv_operating_modes = node.advertiseService("set_operating_modes", &InterbotixRobotXS::robot_srv_set_operating_modes, this);
  srv_motor_gains = node.advertiseService("set_motor_pid_gains", &InterbotixRobotXS::robot_srv_set_motor_pid_gains, this);
  srv_set_registers = node.advertiseService("set_motor_registers", &InterbotixRobotXS::robot_srv_set_motor_registers, this);
  srv_get_registers = node.advertiseService("get_motor_registers", &InterbotixRobotXS::robot_srv_get_motor_registers, this);
}

/// @brief Initialize ROS Timers
void InterbotixRobotXS::robot_init_timers(void)
{
  execute_joint_traj = false;
  if (timer_hz != 0)
    tmr_joint_states = node.createTimer(ros::Duration(1.0/timer_hz), &InterbotixRobotXS::robot_update_joint_states, this);
  tmr_joint_traj = node.createTimer(ros::Duration(0), &InterbotixRobotXS::robot_execute_trajectory, this, true, false);
}

/// @brief Waits until first JointState message is received
void InterbotixRobotXS::robot_wait_for_joint_states(void)
{
  if (timer_hz == 0) return;
  ros::Rate r(10);
  while (ros::ok() && joint_states.name.size() == 0)
  {
    ROS_DEBUG("[xs_sdk::robot_wait_for_joint_states] Waiting for joint states...");
    ros::spinOnce();
    r.sleep();
  }
  ROS_DEBUG("[xs_sdk::robot_wait_for_joint_states] Joint states found. Continuing.");
}

/// @brief ROS Subscriber callback function to command a group of joints
/// @param msg - JointGroupCommand message dictating the joint group to command along with the actual commands
/// @details - refer to the message definition for details
void InterbotixRobotXS::robot_sub_command_group(const interbotix_xs_msgs::JointGroupCommand &msg)
{
  robot_write_commands(msg.name, msg.cmd);
}

/// @brief ROS Subscriber callback function to command a single joint
/// @param msg - JointSingleCommand message dictating the joint to command along with the actual command
/// @details - refer to the message definition for details
void InterbotixRobotXS::robot_sub_command_single(const interbotix_xs_msgs::JointSingleCommand &msg)
{
  robot_write_joint_command(msg.name, msg.cmd);
}

/// @brief ROS Subscriber callback function to command a joint trajectory
/// @param msg - JointTrajectoryCommand message dictating the joint(s) to command along with the desired trajectory
/// @details - refer to the message definition for details
void InterbotixRobotXS::robot_sub_command_traj(const interbotix_xs_msgs::JointTrajectoryCommand &msg)
{
  if (execute_joint_traj)
  {
    ROS_WARN("[xs_sdk] Trajectory rejected since joints are still moving.");
    return;
  }
  if (msg.traj.points.size() < 2)
  {
    ROS_WARN("[xs_sdk] Trajectory has fewer than 2 points. Aborting...");
    return;
  }

  std::vector<std::string> joint_names;
  if (msg.cmd_type == "group")
    joint_names = group_map[msg.name].joint_names;
  else if (msg.cmd_type == "single")
    joint_names.push_back(msg.name);

  if (timer_hz != 0 && msg.traj.points[0].positions.size() == joint_names.size())
  {
    for (size_t i{0}; i < joint_names.size(); i++)
    {
      float expected_state = msg.traj.points[0].positions.at(i);
      float actual_state = joint_states.position.at(js_index_map[joint_names.at(i)]);
      if (!(fabs(expected_state - actual_state) < 0.01))
      {
        ROS_WARN("[xs_sdk] The %s joint is not at the correct initial state.", joint_names.at(i).c_str());
        ROS_WARN("[xs_sdk] Expected state: %.2f, Actual State: %.2f.", expected_state, actual_state);
      }
    }
  }
  joint_traj_cmd = msg;
  execute_joint_traj = true;
  tmr_joint_traj.setPeriod(ros::Duration(0));
  tmr_joint_traj.start();
}

/// @brief ROS Service to torque the joints on the robot on/off
/// @param req - TorqueEnable service message request
/// @param res [out] - TorqueEnable service message response [unused]
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_torque_enable(interbotix_xs_msgs::TorqueEnable::Request &req, interbotix_xs_msgs::TorqueEnable::Response &res)
{
  if (!robot_srv_validate(req.cmd_type, req.name))
    return false;

  robot_torque_enable(req.cmd_type, req.name, req.enable);
  return true;
}

/// @brief ROS Service to reboot the motors on the robot
/// @param req - Reboot service message request
/// @param res [out] - Reboot service message response [unused]
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_reboot_motors(interbotix_xs_msgs::Reboot::Request &req, interbotix_xs_msgs::Reboot::Response &res)
{
  if (!robot_srv_validate(req.cmd_type, req.name))
    return false;

  robot_reboot_motors(req.cmd_type, req.name, req.enable, req.smart_reboot);
  return true;
}

/// @brief ROS Service that allows the user to get information about the robot
/// @param req - RobotInfo service message request
/// @param res [out] - RobotInfo service message response
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_get_robot_info(interbotix_xs_msgs::RobotInfo::Request &req, interbotix_xs_msgs::RobotInfo::Response &res)
{
  if (!robot_srv_validate(req.cmd_type, req.name))
    return false;

  bool urdf_exists = false;
  urdf::Model model;
  urdf::JointConstSharedPtr ptr;
  // Parse the urdf model to get joint limit info
  std::string robot_name = node.getNamespace();
  if (ros::param::has("robot_description"))
  {
    model.initParam(robot_name + "/robot_description");
    urdf_exists = true;
  }
  if (req.cmd_type == "group")
  {
    res.joint_names = group_map[req.name].joint_names;
    res.profile_type = group_map[req.name].profile_type;
    res.mode = group_map[req.name].mode;
  }
  else if (req.cmd_type == "single")
  {
    res.joint_names.push_back(req.name);
    res.profile_type = motor_map[req.name].profile_type;
    res.mode = motor_map[req.name].mode;
  }

  res.num_joints = res.joint_names.size();

  for (auto &name : res.joint_names)
  {
    res.joint_ids.push_back(motor_map[name].motor_id);
    if (gripper_map.count(name) > 0)
    {
      res.joint_sleep_positions.push_back(robot_convert_angular_position_to_linear(name, 0));
      name = gripper_map[name].left_finger;
    }
    else
      res.joint_sleep_positions.push_back(sleep_map[name]);
    res.joint_state_indices.push_back(js_index_map[name]);
    if (urdf_exists)
    {
      ptr = model.getJoint(name);
      res.joint_lower_limits.push_back(ptr->limits->lower);
      res.joint_upper_limits.push_back(ptr->limits->upper);
      res.joint_velocity_limits.push_back(ptr->limits->velocity);
    }
  }

  if (req.name != "all")
  {
    res.name.push_back(req.name);
  }
  else
  {
    for (auto key : group_map)
    {
      res.name.push_back(key.first);
    }
  }

  return true;
}

/// @brief ROS Service that allows the user to change operating modes
/// @param req - OperatingModes service message request
/// @param res [out] - OperatingModes service message response [unused]
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_set_operating_modes(interbotix_xs_msgs::OperatingModes::Request &req, interbotix_xs_msgs::OperatingModes::Response &res)
{
  if (!robot_srv_validate(req.cmd_type, req.name))
    return false;

  robot_set_operating_modes(req.cmd_type, req.name, req.mode, req.profile_type, req.profile_velocity, req.profile_acceleration);
  return true;
}

/// @brief ROS Service that allows the user to set the motor firmware PID gains
/// @param req - MotorGains service message request
/// @param res [out] - MotorGains service message response [unused]
/// @details - refer to the service defintion for details
bool InterbotixRobotXS::robot_srv_set_motor_pid_gains(interbotix_xs_msgs::MotorGains::Request &req, interbotix_xs_msgs::MotorGains::Response &res)
{
  if (!robot_srv_validate(req.cmd_type, req.name))
    return false;

  std::vector<int32_t> gains = {req.kp_pos, req.ki_pos, req.kd_pos, req.k1, req.k2, req.kp_vel, req.ki_vel};
  robot_set_motor_pid_gains(req.cmd_type, req.name, gains);
  return true;
}

/// @brief ROS Service that allows the user to change a specific register to a specific value for multiple motors
/// @param req - RegisterValues service message request
/// @param res [out] - RegisterValues service message response [unused]
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_set_motor_registers(interbotix_xs_msgs::RegisterValues::Request &req, interbotix_xs_msgs::RegisterValues::Response &res)
{
  if (!robot_srv_validate(req.cmd_type, req.name))
    return false;

  robot_set_motor_registers(req.cmd_type, req.name, req.reg, req.value);
  return true;
}

/// @brief ROS Service that allows the user to read a specific register on multiple motors
/// @param req - RegisterValues service message request
/// @param res [out] - RegisterValues service message response
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_get_motor_registers(interbotix_xs_msgs::RegisterValues::Request &req, interbotix_xs_msgs::RegisterValues::Response &res)
{
  if (!robot_srv_validate(req.cmd_type, req.name))
    return false;

  robot_get_motor_registers(req.cmd_type, req.name, req.reg, res.values);
  return true;
}

/// @brief ROS One-Shot Timer used to step through a commanded joint trajectory
/// @param e - TimerEvent message [unused]
void InterbotixRobotXS::robot_execute_trajectory(const ros::TimerEvent &e)
{
  static size_t cntr = 1;

  if (cntr == joint_traj_cmd.traj.points.size())
  {
    execute_joint_traj = false;
    cntr = 1;
    return;
  }
  else
  {
    ros::Duration period = joint_traj_cmd.traj.points[cntr].time_from_start - joint_traj_cmd.traj.points[cntr-1].time_from_start;
    tmr_joint_traj.stop();
    tmr_joint_traj.setPeriod(period - (ros::Time::now() - e.current_real));
    tmr_joint_traj.start();
  }

  if (joint_traj_cmd.cmd_type == "group")
  {
    if (group_map[joint_traj_cmd.name].mode.find("position") != std::string::npos)
    {
      std::vector<float> commands(joint_traj_cmd.traj.points[cntr].positions.begin(), joint_traj_cmd.traj.points[cntr].positions.end());
      robot_write_commands(joint_traj_cmd.name, commands);
    }
    else if (group_map[joint_traj_cmd.name].mode == "velocity")
    {
      std::vector<float> commands(joint_traj_cmd.traj.points[cntr].velocities.begin(), joint_traj_cmd.traj.points[cntr].velocities.end());
      robot_write_commands(joint_traj_cmd.name, commands);
    }
    else if (group_map[joint_traj_cmd.name].mode == "pwm" || group_map[joint_traj_cmd.name].mode == "current")
    {
      std::vector<float> commands(joint_traj_cmd.traj.points[cntr].effort.begin(), joint_traj_cmd.traj.points[cntr].effort.end());
      robot_write_commands(joint_traj_cmd.name, commands);
    }
  }
  else if (joint_traj_cmd.cmd_type == "single")
  {
    if (motor_map[joint_traj_cmd.name].mode.find("position") != std::string::npos)
      robot_write_joint_command(joint_traj_cmd.name, joint_traj_cmd.traj.points[cntr].positions.at(0));
    else if (motor_map[joint_traj_cmd.name].mode == "velocity")
      robot_write_joint_command(joint_traj_cmd.name, joint_traj_cmd.traj.points[cntr].velocities.at(0));
    else if (motor_map[joint_traj_cmd.name].mode == "pwm" || motor_map[joint_traj_cmd.name].mode == "current")
      robot_write_joint_command(joint_traj_cmd.name, joint_traj_cmd.traj.points[cntr].effort.at(0));
  }
  cntr++;
}

/// @brief ROS Timer that reads current states from all the motors and publishes them to the joint_states topic
/// @param e - TimerEvent message [unused]
void InterbotixRobotXS::robot_update_joint_states(const ros::TimerEvent &e)
{
  const char* log;

  sensor_msgs::JointState joint_state_msg;

  std::vector<int32_t> get_current(all_ptr->joint_num, 0);
  std::vector<int32_t> get_velocity(all_ptr->joint_num, 0);
  std::vector<int32_t> get_position(all_ptr->joint_num, 0);
  joint_state_msg.name = all_ptr->joint_names;

  if (dxl_wb.getProtocolVersion() == 2.0f)
  {
    // Execute sync read from all pinged DYNAMIXELs
    if (!dxl_wb.syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                          all_ptr->joint_ids.data(),
                          all_ptr->joint_num,
                          &log))
    {
      ROS_ERROR("[xs_sdk] %s", log);
    }

    // Gets present current of all servos
    if (!dxl_wb.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                all_ptr->joint_ids.data(),
                                all_ptr->joint_num,
                                control_items["Present_Current"]->address,
                                control_items["Present_Current"]->data_length,
                                get_current.data(),
                                &log))
    {
      ROS_ERROR("[xs_sdk] %s", log);
    }             

    // Gets present velocity of all servos
    if (!dxl_wb.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                all_ptr->joint_ids.data(),
                                all_ptr->joint_num,
                                control_items["Present_Velocity"]->address,
                                control_items["Present_Velocity"]->data_length,
                                get_velocity.data(),
                                &log))
    {
      ROS_ERROR("[xs_sdk] %s", log);
    }

    // Gets present position of all servos
    if (!dxl_wb.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                all_ptr->joint_ids.data(),
                                all_ptr->joint_num,
                                control_items["Present_Position"]->address,
                                control_items["Present_Position"]->data_length,
                                get_position.data(),
                                &log))
    {
      ROS_ERROR("[xs_sdk] %s", log);
    }

    uint8_t index = 0;
    for (auto const& id : all_ptr->joint_ids)
    {
      float position = 0;
      float velocity = 0;
      float effort = 0;

      if (strcmp(dxl_wb.getModelName(id), "XL-320") == 0) effort = dxl_wb.convertValue2Load(get_current.at(index));
      else effort = dxl_wb.convertValue2Current(get_current.at(index));
      velocity = dxl_wb.convertValue2Velocity(id, get_velocity.at(index));
      position = dxl_wb.convertValue2Radian(id, get_position.at(index));
      joint_state_msg.effort.push_back(effort);
      joint_state_msg.velocity.push_back(velocity);
      joint_state_msg.position.push_back(position);
      index++;
    }
  }
  else if(dxl_wb.getProtocolVersion() == 1.0f)
  {
    uint16_t length_of_data = control_items["Present_Position"]->data_length +
                              control_items["Present_Velocity"]->data_length +
                              control_items["Present_Current"]->data_length;
    std::vector<uint32_t> get_all_data(length_of_data, 0);

    for (auto const& id : all_ptr->joint_ids)
    {
      if (!dxl_wb.readRegister(id,
                                control_items["Present_Position"]->address,
                                length_of_data,
                                get_all_data.data(),
                                &log))
      {
        ROS_ERROR("[xs_sdk] %s", log);
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
  for (auto const& name : gripper_order)
  {
    joint_state_msg.name.push_back(gripper_map[name].left_finger.c_str());
    joint_state_msg.name.push_back(gripper_map[name].right_finger.c_str());
    float pos = robot_convert_angular_position_to_linear(name, joint_state_msg.position.at(gripper_map[name].js_index));
    joint_state_msg.position.push_back(pos);
    joint_state_msg.position.push_back(-pos);
    joint_state_msg.velocity.push_back(0);
    joint_state_msg.velocity.push_back(0);
    joint_state_msg.effort.push_back(0);
    joint_state_msg.effort.push_back(0);
  }
  // Publish the message to the joint_states topic
  joint_state_msg.header.stamp = ros::Time::now();
  joint_states = joint_state_msg;
  if (pub_states) pub_joint_states.publish(joint_state_msg);
}

/// @brief Checks service call requests for validity
/// @param cmd_type request cmd_type field
/// @param name request name field
/// @returns true if the service call request is valid, false otherwise
/// @details cmd_type must be 'single' or 'group'; name must be in the group_map or motor_map
bool InterbotixRobotXS::robot_srv_validate(const std::string &cmd_type, std::string &name)
{
  if (cmd_type == "group") // if group command...
  {
    if (group_map.count(name) == 1) // if group name is valid, return true
    {
      return true;
    }
    else // otherwise error and return false
    {
      ROS_ERROR("[xs_sdk] Invalid service call. Group '%s' does not exist. Was it added to the motor config file?", name.c_str());
      return false;
    }
  }
  else if (cmd_type == "single") // if single joint command...
  {
    if (motor_map.count(name) == 1) // if joint name is valid, return true
    {
      return true;
    }
    else // otherwise error and return false
    {
      ROS_ERROR("[xs_sdk] Invalid service call. Joint '%s' does not exist. Was it added to the motor config file?", name.c_str());
      return false;
    }
  }
  else // if command type is invalid, error and return false
  {
    ROS_ERROR("[xs_sdk] Invalid service call. cmd_type '%s'. Choices are 'group' or 'single'.", cmd_type.c_str());
    return false;
  }
}
