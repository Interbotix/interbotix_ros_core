#include "interbotix_xs_sdk/xs_sdk_obj.hpp"

/// @brief Constructor for the InterbotixRobotXS
/// @param node_handle - ROS NodeHandle
InterbotixRobotXS::InterbotixRobotXS(
  bool &success,
  const rclcpp::NodeOptions &options)
  :
  rclcpp::Node("xs_sdk", options)
{
  robot_init_parameters();
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
    RCLCPP_ERROR(this->get_logger(), "Failed to find all motors. Shutting down...");
    return;
  }

  if (!robot_load_motor_configs())
  {
    success = false;
    RCLCPP_ERROR(this->get_logger(), "Failed to write configurations to all motors. Shutting down...");
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
  RCLCPP_INFO(this->get_logger(), "Interbotix 'xs_sdk' node is up!");
}

/// @brief Destructor for the InterbotixRobotXS
InterbotixRobotXS::~InterbotixRobotXS(){}

/// @brief Declare all parameters needed by the node
void InterbotixRobotXS::robot_init_parameters(void){
  this->declare_parameter<std::string>("motor_configs", "");
  this->declare_parameter<std::string>("mode_configs", "");
  this->declare_parameter<bool>("load_configs", false);
  this->declare_parameter<std::string>("robot_description", "");
}

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
    RCLCPP_INFO(this->get_logger(), "The operating mode for the '%s' group was changed to %s.", name.c_str(), mode.c_str());
  }
  else if (cmd_type == "single" && motor_map.count(name) > 0)
  {
    robot_set_joint_operating_mode(name, mode, profile_type, profile_velocity, profile_acceleration);
    RCLCPP_INFO(this->get_logger(), "The operating mode for the '%s' joint was changed to %s.", name.c_str(), mode.c_str());
  }
  else if (cmd_type == "group" && group_map.count(name) == 0 || cmd_type == "single" && motor_map.count(name) == 0)
    RCLCPP_WARN(this->get_logger(), "The '%s' joint/group does not exist. Was it added to the motor config file?", name.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Invalid command for argument 'cmd_type' while setting operating mode.");
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
    RCLCPP_DEBUG(this->get_logger(), "ID: %d, torqued off.", motor_map[joint_name].motor_id);
  }

  for (auto const& motor_name:shadow_map[name])
  {
    int32_t drive_mode;
    dxl_wb.itemRead(motor_map[motor_name].motor_id, "Drive_Mode", &drive_mode);
    RCLCPP_DEBUG(this->get_logger(), "ID: %d, read Drive Mode %d.",motor_map[motor_name].motor_id, drive_mode);
    
    if (drive_mode <= 1 && profile_type == "time")
    {
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Drive_Mode", drive_mode + 4);
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, write Drive Mode %d.", motor_map[motor_name].motor_id, drive_mode + 4);
    }
    else if (drive_mode >= 4 && profile_type == "velocity")
    {
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Drive_Mode", drive_mode - 4);
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, write Drive Mode %d.", motor_map[motor_name].motor_id, drive_mode - 4);
    }

    if (mode == "position" || mode == "linear_position")
    {
      dxl_wb.setPositionControlMode(motor_map[motor_name].motor_id);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Velocity", profile_velocity);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Acceleration", profile_acceleration);
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, set poscontrolmode, pv=%i, pa=%i.", motor_map[motor_name].motor_id, profile_velocity, profile_acceleration);
    }
    else if (mode == "ext_position")
    {
      dxl_wb.setExtendedPositionControlMode(motor_map[motor_name].motor_id);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Velocity", profile_velocity);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Acceleration", profile_acceleration);
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, set extposcontrolmode, pv=%i, pa=%i.", motor_map[motor_name].motor_id, profile_velocity, profile_acceleration);

    }
    else if (mode == "velocity")
    {
      dxl_wb.setVelocityControlMode(motor_map[motor_name].motor_id);
      dxl_wb.itemWrite(motor_map[motor_name].motor_id, "Profile_Acceleration", profile_acceleration);
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, set velcontrolmode, pa=%i.", motor_map[motor_name].motor_id, profile_acceleration);
    }
    else if (mode == "pwm")
    {
      dxl_wb.setPWMControlMode(motor_map[motor_name].motor_id);
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, set pwmcontrolmode.", motor_map[motor_name].motor_id);
    }

    else if (mode == "current")
    {
      dxl_wb.setCurrentControlMode(motor_map[motor_name].motor_id);
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, set currentcontrolmode", motor_map[motor_name].motor_id);
    }
    else if (mode == "current_based_position")
    {
      dxl_wb.setCurrentBasedPositionControlMode(motor_map[motor_name].motor_id);
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, set currentcontrolmode", motor_map[motor_name].motor_id);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid command for argument 'mode' while setting the operating mode for the %s motor.", motor_name.c_str());
      continue;
    }
    motor_map[motor_name].mode = mode;
    motor_map[motor_name].profile_type = profile_type;
  }

  for (auto const& joint_name:sister_map[name])
  {
    dxl_wb.torque(motor_map[joint_name].motor_id, true);
    RCLCPP_DEBUG(this->get_logger(), "ID: %d, torqued on.", motor_map[joint_name].motor_id);
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
    if (enable) RCLCPP_INFO(this->get_logger(), "The '%s' group was torqued on.", name.c_str());
    else RCLCPP_INFO(this->get_logger(), "The '%s' group was torqued off.", name.c_str());
  }
  else if (cmd_type == "single" && motor_map.count(name) > 0)
  {
    dxl_wb.torque(motor_map[name].motor_id, enable);
    if (enable) RCLCPP_INFO(this->get_logger(), "The '%s' joint was torqued on.", name.c_str());
    else RCLCPP_INFO(this->get_logger(), "The '%s' joint was torqued off.", name.c_str());
  }
  else if (cmd_type == "group" && group_map.count(name) == 0 || cmd_type == "single" && motor_map.count(name) == 0)
    RCLCPP_WARN(this->get_logger(), "The '%s' joint/group does not exist. Was it added to the motor config file?", name.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Invalid command for argument 'cmd_type' while torquing joints.");
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
      RCLCPP_INFO(this->get_logger(), "The '%s' joint was rebooted.", joint_name.c_str());
      if (enable) joints_to_torque.push_back(joint_name);
    }
    if (!smart_reboot)
      RCLCPP_INFO(this->get_logger(), "The '%s' group was rebooted.", name.c_str());
  }
  else if (cmd_type == "single" && motor_map.count(name) > 0)
  {
    dxl_wb.reboot(motor_map[name].motor_id);
    RCLCPP_INFO(this->get_logger(), "The '%s' joint was rebooted.", name.c_str());
    if (enable) joints_to_torque.push_back(name);
  }
  else if (cmd_type == "group" && group_map.count(name) == 0 || cmd_type == "single" && motor_map.count(name) == 0)
    RCLCPP_WARN(this->get_logger(), "The '%s' joint/group does not exist. Was it added to the motor config file?", name.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Invalid command for argument 'cmd_type' while rebooting motors.");

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
  std::string mode = group_map[name].mode;
  int32_t dynamixel_commands[commands.size()];

  if (mode == "position" || mode == "ext_position" || mode == "current_based_position" || mode == "linear_position")
  {
    for (size_t i{0}; i < commands.size(); i++)
    {
      if (mode == "linear_position")
        commands.at(i) = robot_convert_linear_position_to_radian(group_map[name].joint_names.at(i), commands.at(i));
      dynamixel_commands[i] = dxl_wb.convertRadian2Value(group_map[name].joint_ids.at(i), commands.at(i));
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing %s command %d.", group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, group_map[name].joint_ids.data(), group_map[name].joint_num, dynamixel_commands, 1);
  }
  else if (mode == "velocity")
  {
    for (size_t i{0}; i < commands.size(); i++)
    {
      dynamixel_commands[i] = dxl_wb.convertVelocity2Value(group_map[name].joint_ids.at(i), commands.at(i));
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing %s command %d.", group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, group_map[name].joint_ids.data(), group_map[name].joint_num, dynamixel_commands, 1);
  }
  else if (mode == "current")
  {
    for (size_t i{0}; i < commands.size(); i++)
    {
      dynamixel_commands[i] = dxl_wb.convertCurrent2Value(commands.at(i));
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing %s command %d.", group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, group_map[name].joint_ids.data(), group_map[name].joint_num, dynamixel_commands, 1);
  }
  else if (mode == "pwm")
  {
    for (size_t i{0}; i < commands.size(); i++)
    {
      dynamixel_commands[i] = int32_t(commands.at(i));
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing %s command %d.", group_map[name].joint_ids.at(i), mode.c_str(), dynamixel_commands[i]);
    }
    dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_PWM, group_map[name].joint_ids.data(), group_map[name].joint_num, dynamixel_commands, 1);
  }
  else
    RCLCPP_ERROR(this->get_logger(), "Invalid command for argument 'mode' while commanding joint group.");
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
    RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing %s command %f.", motor_map[name].motor_id, mode.c_str(), command);
    dxl_wb.goalPosition(motor_map[name].motor_id, command);
  }
  else if (mode == "velocity")
  {
    RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing %s command %f.", motor_map[name].motor_id, mode.c_str(), command);
    dxl_wb.goalVelocity(motor_map[name].motor_id, command);
  }
  else if (mode == "current")
  {
    RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing %s command %f.", motor_map[name].motor_id, mode.c_str(), command);
    dxl_wb.itemWrite(motor_map[name].motor_id, "Goal_Current", dxl_wb.convertCurrent2Value(command));
  }
  else if (mode == "pwm")
  {
    RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing %s command %f.", motor_map[name].motor_id, mode.c_str(), command);
    dxl_wb.itemWrite(motor_map[name].motor_id, "Goal_PWM", int32_t(command));
  }
  else
    RCLCPP_ERROR(this->get_logger(), "Invalid command for argument 'mode' while commanding joint.");
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
    RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing gains:", motor_map[name].motor_id);
    RCLCPP_DEBUG(this->get_logger(), "        Pos_P: %i", gains.at(0));
    RCLCPP_DEBUG(this->get_logger(), "        Pos_I: %i", gains.at(1));
    RCLCPP_DEBUG(this->get_logger(), "        Pos_D: %i", gains.at(2));
    RCLCPP_DEBUG(this->get_logger(), "        FF_1: %i", gains.at(3));
    RCLCPP_DEBUG(this->get_logger(), "        FF_2: %i", gains.at(4));
    RCLCPP_DEBUG(this->get_logger(), "        Vel_P: %i", gains.at(5));
    RCLCPP_DEBUG(this->get_logger(), "        Vel_I: %i", gains.at(6));
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
    RCLCPP_DEBUG(this->get_logger(), "ID: %d, writing reg: %s, value: %d.", motor_map[name].motor_id, reg.c_str(), value);
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
    RCLCPP_ERROR(this->get_logger(), "Could not get '%s' Item Info. Did you spell the register name correctly?", reg.c_str());
    return;
  }

  for (auto const& name : names)
  {
    int32_t value = 0;
    const char *log;
    if (!dxl_wb.itemRead(motor_map[name].motor_id, reg.c_str(), &value, &log))
    {
      RCLCPP_ERROR(this->get_logger(), "%s", log);
      return;
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "ID: %d, reading reg: %s, value: %d.", motor_map[name].motor_id, reg.c_str(), value);
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
    RCLCPP_DEBUG(this->get_logger(), "ID: %ld, got joint state.", js_index_map[joint_name]);
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
  RCLCPP_DEBUG(this->get_logger(), "ID: %ld, got joint state.", js_index_map[name]);
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
  this->get_parameter("motor_configs", motor_configs_file);
  try
  {
    motor_configs = YAML::LoadFile(motor_configs_file.c_str());
  }
  catch (YAML::BadFile &error)
  {
    RCLCPP_ERROR(this->get_logger(), "Motor Config file was not found or has a bad format. Shutting down...");
    RCLCPP_ERROR(this->get_logger(), "YAML Error: '%s'", error.what());
    return false;
  }
  if (motor_configs.IsNull())
  {
    RCLCPP_ERROR(this->get_logger(), "Motor Config file was not found. Shutting down...");
    return false;
  }

  this->get_parameter("mode_configs", mode_configs_file);
  try
  {
    mode_configs = YAML::LoadFile(mode_configs_file.c_str());
  }
  catch (YAML::BadFile &error)
  {
    RCLCPP_ERROR(this->get_logger(), "Motor Config file was not found or has a bad format. Shutting down...");
    RCLCPP_ERROR(this->get_logger(), "YAML Error: '%s'", error.what());
    return false;
  }
  if (mode_configs.IsNull())
    RCLCPP_INFO(this->get_logger(), "Mode Config file is empty.");

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

  RCLCPP_INFO(this->get_logger(), "Successfully retrieved motor configs from %s.", motor_configs_file.c_str());
  return true;
}

/// @brief Initializes the port to communicate with the Dynamixel servos
/// @param <bool> [out] - True if the port was successfully opened; False otherwise
bool InterbotixRobotXS::robot_init_port(void)
{
  if (!dxl_wb.init(port.c_str(), BAUDRATE))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open port at %s. Shutting down...", port.c_str());
    return false;
  }
  return true;
}

/// @brief Pings all motors to make sure they can be found
/// @param <bool> [out] - True if all motors were found; False otherwise
bool InterbotixRobotXS::robot_ping_motors(void)
{
  for (auto const& motor:motor_map)
  {
    uint16_t model_number = 0;
    if(!dxl_wb.ping(motor.second.motor_id, &model_number))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Can't find Dynamixel ID '%d',\tJoint Name : '%s'", 
        motor.second.motor_id, motor.first.c_str());
      return false;
    }
    else 
      RCLCPP_INFO(this->get_logger(),
        "Found Dynamixel ID : %d,\tModel Number : %d,\tJoint Name : %s", 
        motor.second.motor_id, model_number, motor.first.c_str());
    dxl_wb.torque(motor.second.motor_id, false);
  }
  return true;
}

/// @brief Writes some 'startup' EEPROM register values to the Dynamixel servos
/// @param <bool> [out] - True if all register values were written successfully; False otherwise
bool InterbotixRobotXS::robot_load_motor_configs(void)
{
  bool load_configs;
  this->get_parameter("load_configs", load_configs);

  if (load_configs)
  {
    for (auto const& motor_info:motor_info_vec)
    {
      if (!dxl_wb.itemWrite(motor_info.motor_id, motor_info.reg.c_str(), motor_info.value))
      {
        RCLCPP_ERROR(this->get_logger(),
          "[xs_sdk] Failed to write value[%d] on items[%s] to [ID : %d]",
          motor_info.value, motor_info.reg.c_str(), motor_info.motor_id);
        return false;
      }
    }
  }
  else
    RCLCPP_INFO(this->get_logger(), "Skipping Load Configs...");
  return true;
}

/// @brief Retrieves information about 'Goal_XXX' and 'Present_XXX' registers
/// @details - Info includes a register's name, address, and data length
void InterbotixRobotXS::robot_init_controlItems(void)
{
  uint8_t motor_id = motor_map.begin()->second.motor_id;

  const ControlItem *goal_position = dxl_wb.getItemInfo(motor_id, "Goal_Position");
  if (!goal_position)
    RCLCPP_ERROR(this->get_logger(), "Could not get 'Goal_Position' Item Info");

  const ControlItem *goal_velocity = dxl_wb.getItemInfo(motor_id, "Goal_Velocity");
  if (!goal_velocity) goal_velocity = dxl_wb.getItemInfo(motor_id, "Moving_Speed");
  if (!goal_velocity)
    RCLCPP_ERROR(this->get_logger(), "Could not get 'Goal_Velocity' or 'Moving_Speed' Item Info");

  const ControlItem *goal_current = NULL;
  for (auto const& motor:motor_map)
  {
    goal_current = dxl_wb.getItemInfo(motor.second.motor_id, "Goal_Current");
    if (goal_current)
      break;
  }
  if (!goal_current)
    RCLCPP_INFO(
      this->get_logger(),
      "Could not get 'Goal_Current' Item Info. This message can be "
      "ignored if none of the robot's motors support current control.");

  const ControlItem *goal_pwm = dxl_wb.getItemInfo(motor_id, "Goal_PWM");
  if (!goal_pwm)
    RCLCPP_ERROR(this->get_logger(), "Could not get 'Goal_PWM' Item Info");

  const ControlItem *present_position = dxl_wb.getItemInfo(motor_id, "Present_Position");
  if (!present_position)
    RCLCPP_ERROR(this->get_logger(), "Could not get 'Present_Position' Item Info");

  const ControlItem *present_velocity = dxl_wb.getItemInfo(motor_id, "Present_Velocity");
  if (!present_velocity) present_velocity = dxl_wb.getItemInfo(motor_id, "Present_Speed");
  if (!present_velocity)
    RCLCPP_ERROR(this->get_logger(), "Could not get 'Present_Velocity' or 'Present_Speed' Item Info");

  const ControlItem *present_current = dxl_wb.getItemInfo(motor_id, "Present_Current");
  if (!present_current) present_current = dxl_wb.getItemInfo(motor_id, "Present_Load");
  if (!present_current)
    RCLCPP_ERROR(this->get_logger(), "Could not get 'Present_Current' or 'Present_Load' Item Info");

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
    RCLCPP_ERROR(this->get_logger(), "Failed to add SyncWriteHandler for Goal_Position.");

  if (!dxl_wb.addSyncWriteHandler(control_items["Goal_Velocity"]->address, control_items["Goal_Velocity"]->data_length))
    RCLCPP_ERROR(this->get_logger(), "Failed to add SyncWriteHandler for Goal_Velocity.");

  // only add a SyncWriteHandler for 'Goal_Current' if the register actually exists!
  if (control_items["Goal_Current"])
  {
    if (!dxl_wb.addSyncWriteHandler(control_items["Goal_Current"]->address, control_items["Goal_Current"]->data_length))
      RCLCPP_ERROR(this->get_logger(), "Failed to add SyncWriteHandler for Goal_Current.");
  }
  else
    RCLCPP_INFO(this->get_logger(), "SyncWriteHandler for Goal_Current not added as it's not supported.");

  if (!dxl_wb.addSyncWriteHandler(control_items["Goal_PWM"]->address, control_items["Goal_PWM"]->data_length))
    RCLCPP_ERROR(this->get_logger(), "Failed to add SyncWriteHandler for Goal_PWM.");

  if (dxl_wb.getProtocolVersion() == 2.0f)
  {
    uint16_t start_address = std::min(control_items["Present_Position"]->address, control_items["Present_Current"]->address);
    /*
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */
    // uint16_t read_length = control_items["Present_Position"]->data_length + control_items["Present_Velocity"]->data_length + control_items["Present_Current"]->data_length;
    uint16_t read_length = control_items["Present_Position"]->data_length + control_items["Present_Velocity"]->data_length + control_items["Present_Current"]->data_length+2;
    if (!dxl_wb.addSyncReadHandler(start_address, read_length))
      RCLCPP_ERROR(this->get_logger(), "Failed to add SyncReadHandler");
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
    pub_joint_states = this->create_publisher<sensor_msgs::msg::JointState>(js_topic, 10);
}

/// @brief Initialize ROS Subscribers
void InterbotixRobotXS::robot_init_subscribers(void)
{
  using namespace std::placeholders;
  sub_command_group = this->create_subscription<JointGroupCommand>("commands/joint_group", 10, std::bind(&InterbotixRobotXS::robot_sub_command_group, this, _1));
  sub_command_single = this->create_subscription<JointSingleCommand>("commands/joint_single", 10, std::bind(&InterbotixRobotXS::robot_sub_command_single, this, _1));
  sub_command_traj = this->create_subscription<JointTrajectoryCommand>("commands/joint_trajectory", 10, std::bind(&InterbotixRobotXS::robot_sub_command_traj, this, _1));
}

/// @brief Initialize ROS Services
void InterbotixRobotXS::robot_init_services(void)
{
  using namespace std::placeholders;
  srv_torque_enable = this->create_service<TorqueEnable>("torque_enable", std::bind(&InterbotixRobotXS::robot_srv_torque_enable, this, _1, _2, _3));
  srv_reboot_motors = this->create_service<Reboot>("reboot_motors", std::bind(&InterbotixRobotXS::robot_srv_reboot_motors, this, _1, _2, _3));
  srv_get_robot_info = this->create_service<RobotInfo>("get_robot_info", std::bind(&InterbotixRobotXS::robot_srv_get_robot_info, this, _1, _2, _3));
  srv_operating_modes = this->create_service<OperatingModes>("set_operating_modes", std::bind(&InterbotixRobotXS::robot_srv_set_operating_modes, this, _1, _2, _3));
  srv_motor_gains = this->create_service<MotorGains>("set_motor_pid_gains", std::bind(&InterbotixRobotXS::robot_srv_set_motor_pid_gains, this, _1, _2, _3));
  srv_set_registers = this->create_service<RegisterValues>("set_motor_registers", std::bind(&InterbotixRobotXS::robot_srv_set_motor_registers, this, _1, _2, _3));
  srv_get_registers = this->create_service<RegisterValues>("get_motor_registers", std::bind(&InterbotixRobotXS::robot_srv_get_motor_registers, this, _1, _2, _3));
}

/// @brief Initialize ROS Timers
void InterbotixRobotXS::robot_init_timers(void)
{
  execute_joint_traj = false;
  using namespace std::chrono_literals;
  if (timer_hz != 0)
  {
    // create a timer that updates the joint states with a period of 1/timer_hz
    std::chrono::milliseconds update_rate_in_ms = std::chrono::milliseconds(int(1.0/(timer_hz)*1000.0));
    tmr_joint_states = this->create_wall_timer(
      update_rate_in_ms, 
      std::bind(&InterbotixRobotXS::robot_update_joint_states,
        this));
  }
}

/// @brief Waits until first JointState message is received
void InterbotixRobotXS::robot_wait_for_joint_states(void)
{
  if (timer_hz == 0) return;
  rclcpp::Rate r(10);
  while (rclcpp::ok() && joint_states.name.size() == 0)
  {
    rclcpp::spin_some(this->get_node_base_interface());
    r.sleep();
  }
}

/// @brief ROS Subscriber callback function to command a group of joints
/// @param msg - JointGroupCommand message dictating the joint group to command along with the actual commands
/// @details - refer to the message definition for details
void InterbotixRobotXS::robot_sub_command_group(const JointGroupCommand::SharedPtr msg)
{
  robot_write_commands(msg->name, msg->cmd);
}

/// @brief ROS Subscriber callback function to command a single joint
/// @param msg - JointSingleCommand message dictating the joint to command along with the actual command
/// @details - refer to the message definition for details
void InterbotixRobotXS::robot_sub_command_single(const JointSingleCommand::SharedPtr msg)
{
  robot_write_joint_command(msg->name, msg->cmd);
}

/// @brief ROS Subscriber callback function to command a joint trajectory
/// @param msg - JointTrajectoryCommand message dictating the joint(s) to command along with the desired trajectory
/// @details - refer to the message definition for details
void InterbotixRobotXS::robot_sub_command_traj(const JointTrajectoryCommand::SharedPtr msg)
{
  using namespace std::chrono_literals;
  if (execute_joint_traj)
  {
    RCLCPP_WARN(this->get_logger(), "Trajectory rejected since joints are still moving.");
    return;
  }
  if (msg->traj.points.size() < 2)
  {
    RCLCPP_WARN(this->get_logger(), "Trajectory has fewer than 2 points. Aborting...");
    return;
  }

  std::vector<std::string> joint_names;
  if (msg->cmd_type == "group")
    joint_names = group_map[msg->name].joint_names;
  else if (msg->cmd_type == "single")
    joint_names.push_back(msg->name);

  if (timer_hz != 0 && msg->traj.points[0].positions.size() == joint_names.size())
  {
    for (size_t i{0}; i < joint_names.size(); i++)
    {
      float expected_state = msg->traj.points[0].positions.at(i);
      float actual_state = joint_states.position.at(js_index_map[joint_names.at(i)]);
      if (!(fabs(expected_state - actual_state) < 0.01))
      {
        RCLCPP_WARN(this->get_logger(), "The %s joint is not at the correct initial state.", joint_names.at(i).c_str());
        RCLCPP_WARN(this->get_logger(), "Expected state: %.2f, Actual State: %.2f.", expected_state, actual_state);
      }
    }
  }
  joint_traj_cmd = msg;
  execute_joint_traj = true;

  // create timer that immediately triggers callback
  tmr_joint_traj = create_wall_timer(
    0s,
    std::bind(&InterbotixRobotXS::robot_execute_trajectory,
      this));
}

/// @brief ROS Service to torque the joints on the robot on/off
/// @param req - TorqueEnable service message request
/// @param res [out] - TorqueEnable service message response [unused]
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_torque_enable(const std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<TorqueEnable::Request> req, std::shared_ptr<TorqueEnable::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name))
    return false;

  robot_torque_enable(req->cmd_type, req->name, req->enable);
  return true;
}

/// @brief ROS Service to reboot the motors on the robot
/// @param req - Reboot service message request
/// @param res [out] - Reboot service message response [unused]
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_reboot_motors(const std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<Reboot::Request> req, std::shared_ptr<Reboot::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name))
    return false;

  robot_reboot_motors(req->cmd_type, req->name, req->enable, req->smart_reboot);
  return true;
}

/// @brief ROS Service that allows the user to get information about the robot
/// @param req - RobotInfo service message request
/// @param res [out] - RobotInfo service message response
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_get_robot_info(const std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<RobotInfo::Request> req, std::shared_ptr<RobotInfo::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name))
    return false;
  bool urdf_exists = false;
  urdf::Model model;
  urdf::JointConstSharedPtr ptr;
  // Parse the urdf model to get joint limit info
  std::string robot_name = this->get_namespace();
  std::string robot_description;
  this->get_parameter("robot_description", robot_description);
  if (!robot_description.empty())
  {
    model.initString(robot_description);
    urdf_exists = true;
  }
  if (req->cmd_type == "group")
  {
    res->joint_names = group_map[req->name].joint_names;
    res->profile_type = group_map[req->name].profile_type;
    res->mode = group_map[req->name].mode;
  }
  else if (req->cmd_type == "single")
  {
    res->joint_names.push_back(req->name);
    res->profile_type = motor_map[req->name].profile_type;
    res->mode = motor_map[req->name].mode;
  }

  res->num_joints = res->joint_names.size();

  for (auto &name : res->joint_names)
  {
    res->joint_ids.push_back(motor_map[name].motor_id);
    if (gripper_map.count(name) > 0)
    {
      res->joint_sleep_positions.push_back(robot_convert_angular_position_to_linear(name, 0));
      name = gripper_map[name].left_finger;
    }
    else
      res->joint_sleep_positions.push_back(sleep_map[name]);
    res->joint_state_indices.push_back(js_index_map[name]);
    if (urdf_exists)
    {
      ptr = model.getJoint(name);
      res->joint_lower_limits.push_back(ptr->limits->lower);
      res->joint_upper_limits.push_back(ptr->limits->upper);
      res->joint_velocity_limits.push_back(ptr->limits->velocity);
    }
  }
}

/// @brief ROS Service that allows the user to change operating modes
/// @param req - OperatingModes service message request
/// @param res [out] - OperatingModes service message response [unused]
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_set_operating_modes(const std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<OperatingModes::Request> req, std::shared_ptr<OperatingModes::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name))
    return false;

  robot_set_operating_modes(req->cmd_type, req->name, req->mode, req->profile_type, req->profile_velocity, req->profile_acceleration);
  return true;
}

/// @brief ROS Service that allows the user to set the motor firmware PID gains
/// @param req - MotorGains service message request
/// @param res [out] - MotorGains service message response [unused]
/// @details - refer to the service defintion for details
bool InterbotixRobotXS::robot_srv_set_motor_pid_gains(const std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<MotorGains::Request> req, std::shared_ptr<MotorGains::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name))
    return false;

  std::vector<int32_t> gains = {req->kp_pos, req->ki_pos, req->kd_pos, req->k1, req->k2, req->kp_vel, req->ki_vel};
  robot_set_motor_pid_gains(req->cmd_type, req->name, gains);
  return true;
}

/// @brief ROS Service that allows the user to change a specific register to a specific value for multiple motors
/// @param req - RegisterValues service message request
/// @param res [out] - RegisterValues service message response [unused]
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_set_motor_registers(const std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<RegisterValues::Request> req, std::shared_ptr<RegisterValues::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name))
    return false;

  robot_set_motor_registers(req->cmd_type, req->name, req->reg, req->value);
  return true;
}

/// @brief ROS Service that allows the user to read a specific register on multiple motors
/// @param req - RegisterValues service message request
/// @param res [out] - RegisterValues service message response
/// @details - refer to the service definition for details
bool InterbotixRobotXS::robot_srv_get_motor_registers(const std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<RegisterValues::Request> req, std::shared_ptr<RegisterValues::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name))
    return false;

  robot_get_motor_registers(req->cmd_type, req->name, req->reg, res->values);
  return true;
}

/// @brief ROS One-Shot Timer used to step through a commanded joint trajectory
void InterbotixRobotXS::robot_execute_trajectory()
{
  // get the current real time for this callback execution
  rclcpp::Time current_real = this->get_clock()->now();
  using namespace std::chrono_literals;

  static size_t cntr = 1;
  RCLCPP_DEBUG(this->get_logger(), "Executing trajectory step %li/%li.", cntr, joint_traj_cmd->traj.points.size());

  // check if end of trajectory has been reached
  // if done, reset counter, set execution status bool to false, and cancel the
  //  trajectory execution timer.
  // if not done, cancel the timer (pseudo one-shot), and start another.
  if (cntr == joint_traj_cmd->traj.points.size())
  {
    if (!tmr_joint_traj->is_canceled())
      tmr_joint_traj->cancel();
    execute_joint_traj = false;
    cntr = 1;
    RCLCPP_DEBUG(this->get_logger(), "Reached end of trajectory.");
    return;
  }
  else
  {
    // cancel trajectory timer
    if (!tmr_joint_traj->is_canceled())
      tmr_joint_traj->cancel();
    
    // get the length of time the timer should be if perfect
    rclcpp::Duration period = std::chrono::nanoseconds(
      joint_traj_cmd->traj.points[cntr].time_from_start.nanosec 
        - joint_traj_cmd->traj.points[cntr-1].time_from_start.nanosec);
    
    // create new timer with the actual length of time it should execute
    //  (period - (now - start_of_callback))
    tmr_joint_traj = this->create_wall_timer(
      std::chrono::nanoseconds(
        period.nanoseconds() - (this->get_clock()->now().nanoseconds() - current_real.nanoseconds())),
      std::bind(&InterbotixRobotXS::robot_execute_trajectory,
        this));
  }

  // write commands to the motors depending on cmd_type and mode
  if (joint_traj_cmd->cmd_type == "group")
  {
    if (group_map[joint_traj_cmd->name].mode.find("position") != std::string::npos)
    {
      std::vector<float> commands(joint_traj_cmd->traj.points[cntr].positions.begin(), joint_traj_cmd->traj.points[cntr].positions.end());
      robot_write_commands(joint_traj_cmd->name, commands);
    }
    else if (group_map[joint_traj_cmd->name].mode == "velocity")
    {
      std::vector<float> commands(joint_traj_cmd->traj.points[cntr].velocities.begin(), joint_traj_cmd->traj.points[cntr].velocities.end());
      robot_write_commands(joint_traj_cmd->name, commands);
    }
    else if (group_map[joint_traj_cmd->name].mode == "pwm" || group_map[joint_traj_cmd->name].mode == "current")
    {
      std::vector<float> commands(joint_traj_cmd->traj.points[cntr].effort.begin(), joint_traj_cmd->traj.points[cntr].effort.end());
      robot_write_commands(joint_traj_cmd->name, commands);
    }
  }
  else if (joint_traj_cmd->cmd_type == "single")
  {
    if (motor_map[joint_traj_cmd->name].mode.find("position") != std::string::npos)
      robot_write_joint_command(joint_traj_cmd->name, joint_traj_cmd->traj.points[cntr].positions.at(0));
    else if (motor_map[joint_traj_cmd->name].mode == "velocity")
      robot_write_joint_command(joint_traj_cmd->name, joint_traj_cmd->traj.points[cntr].velocities.at(0));
    else if (motor_map[joint_traj_cmd->name].mode == "pwm" || motor_map[joint_traj_cmd->name].mode == "current")
      robot_write_joint_command(joint_traj_cmd->name, joint_traj_cmd->traj.points[cntr].effort.at(0));
  }
  cntr++;
}

/// @brief ROS Timer that reads current states from all the motors and publishes them to the joint_states topic
void InterbotixRobotXS::robot_update_joint_states()
{
  bool result = false;
  const char* log;

  sensor_msgs::msg::JointState joint_state_msg;

  std::vector<int32_t> get_current(all_ptr->joint_num, 0);
  std::vector<int32_t> get_velocity(all_ptr->joint_num, 0);
  std::vector<int32_t> get_position(all_ptr->joint_num, 0);
  joint_state_msg.name = all_ptr->joint_names;

  if (dxl_wb.getProtocolVersion() == 2.0f)
  {
    // Checks if data can be sent properly
    if (!dxl_wb.syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                          all_ptr->joint_ids.data(),
                          all_ptr->joint_num,
                          &log))
    {
      RCLCPP_ERROR(this->get_logger(), "%s", log);
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
      RCLCPP_ERROR(this->get_logger(), "%s", log);
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
      RCLCPP_ERROR(this->get_logger(), "%s", log);
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
      RCLCPP_ERROR(this->get_logger(), "%s", log);
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
        RCLCPP_ERROR(this->get_logger(), "%s", log);
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
  joint_state_msg.header.stamp = this->get_clock()->now();
  joint_states = joint_state_msg;
  if (pub_states) pub_joint_states->publish(joint_state_msg);
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
      RCLCPP_ERROR(this->get_logger(), "service call. Group '%s' does not exist. Was it added to the motor config file?", name.c_str());
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
      RCLCPP_ERROR(this->get_logger(), "service call. Joint '%s' does not exist. Was it added to the motor config file?", name.c_str());
      return false;
    }
  }
  else // if command type is invalid, error and return false
  {
    RCLCPP_ERROR(this->get_logger(), "service call. cmd_type '%s'. Choices are 'group' or 'single'.", cmd_type.c_str());
    return false;
  }
}
