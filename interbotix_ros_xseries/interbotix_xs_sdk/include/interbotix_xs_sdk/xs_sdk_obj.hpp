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

#ifndef INTERBOTIX_XS_SDK__XS_SDK_OBJ_HPP_
#define INTERBOTIX_XS_SDK__XS_SDK_OBJ_HPP_

#include <string>
#include <regex>
#include <bitset>
#include <vector>
#include <algorithm>
#include <memory>
#include <unordered_map>

#include "urdf/model.h"
#include "yaml-cpp/yaml.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "interbotix_xs_msgs/srv/reboot.hpp"
#include "interbotix_xs_msgs/srv/robot_info.hpp"
#include "interbotix_xs_msgs/srv/motor_gains.hpp"
#include "interbotix_xs_msgs/srv/torque_enable.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"
#include "interbotix_xs_msgs/srv/register_values.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
#include "interbotix_xs_msgs/msg/joint_trajectory_command.hpp"

namespace interbotix_xs
{

// All motors are preset to 1M baud
#define DEFAULT_BAUDRATE 1000000

// Udev rule creates a symlink with this name
#define DEFAULT_PORT "/dev/ttyDXL"

// Write goal positions [rad] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0

// Write goal velocities [rad/s] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// Write goal currents [mA] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT 2

// Write goal pwms to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_PWM 3

// Read current joint states for multiple motors at the same time
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// Default motor operating mode is 'position'
inline static const std::string DEFAULT_OP_MODE = "position";

// Default motor profile type is 'velocity' (as opposed to 'time')
inline static const std::string DEFAULT_PROF_TYPE = "velocity";

// Allow joint velocity to be infinite when in position control mode - makes robot very reactive to
// joint commands
static const int32_t DEFAULT_PROF_VEL = 0;

// Allow joint acceleration to be infinite when in position control mode - makes robot very
// reactive to joint commands
static const int32_t DEFAULT_PROF_ACC = 0;

// Torque motor on by default
static const bool TORQUE_ENABLE = true;

// Get motor configurations by default
static bool LOAD_CONFIGS = true;

// Constants for operating modes
inline static const std::string MODE_PWM = "pwm";
inline static const std::string MODE_POSITION = "position";
inline static const std::string MODE_EXT_POSITION = "ext_postition";
inline static const std::string MODE_CURRENT_BASED_POSITION = "current_based_position";
inline static const std::string MODE_LINEAR_POSITION = "linear_position";
inline static const std::string MODE_VELOCITY = "velocity";
inline static const std::string MODE_CURRENT = "current";

// constants for command types
inline static const std::string CMD_TYPE_GROUP = "group";
inline static const std::string CMD_TYPE_SINGLE = "single";

// constants for profile types
inline static const std::string PROFILE_VELOCITY = "velocity";
inline static const std::string PROFILE_TIME = "time";

// simplify message and service usage
using MotorGains = interbotix_xs_msgs::srv::MotorGains;
using OperatingModes = interbotix_xs_msgs::srv::OperatingModes;
using Reboot = interbotix_xs_msgs::srv::Reboot;
using RegisterValues = interbotix_xs_msgs::srv::RegisterValues;
using RobotInfo = interbotix_xs_msgs::srv::RobotInfo;
using TorqueEnable = interbotix_xs_msgs::srv::TorqueEnable;
using JointGroupCommand = interbotix_xs_msgs::msg::JointGroupCommand;
using JointSingleCommand = interbotix_xs_msgs::msg::JointSingleCommand;
using JointTrajectoryCommand = interbotix_xs_msgs::msg::JointTrajectoryCommand;

// Struct to hold multiple joints that represent a group
struct JointGroup
{
  // Names of all joints in the group
  std::vector<std::string> joint_names;
  // Dynamixel ID of all joints in the group
  std::vector<uint8_t> joint_ids;
  // Number of joints in the group
  uint8_t joint_num;
  // Operating Mode for all joints in the group
  std::string mode;
  // Profile Type ('velocity' or 'time') for all joints in the group
  std::string profile_type;
};

// Struct to hold data on a single motor
struct MotorState
{
  // Dynamixel ID of the motor
  uint8_t motor_id;
  // Operating Mode of the motor
  std::string mode;
  // Profile Type ('velocity' or 'time') for the motor
  std::string profile_type;
};

// Struct to hold data on an Interbotix Gripper
struct Gripper
{
  // Index in the published JointState message 'name' list belonging to the gripper motor
  size_t js_index;
  // Distance [m] from the motor horn's center to its edge
  float horn_radius;
  // Distance [m] from the edge of the motor horn to a finger
  float arm_length;
  // Name of the 'left_finger' joint as defined in the URDF (if present)
  std::string left_finger;
  // Name of the 'right_finger' joint as defined in the URDF (if present)
  std::string right_finger;
};

// Struct to hold a desired register value for a given motor
struct MotorInfo
{
  // Dynamixel ID of a motor
  uint8_t motor_id;
  // Register name
  std::string reg;
  // Value to write to the above register for the specified motor
  int32_t value;
};

// Interbotix Core Class to build any type of Dynamixel-based robot
class InterbotixRobotXS : public rclcpp::Node
{
public:
  /// @brief Constructor for the InterbotixRobotXS
  /// @param options ROS NodeOptions
  explicit InterbotixRobotXS(
    bool & success,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor for the InterbotixRobotXS
  ~InterbotixRobotXS();

  /// @brief Set the operating mode for a specific group of motors or a single motor
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if changing the operating mode for a group of motors
  ///   or 'CMD_TYPE_SINGLE' if changing the operating mode for a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param mode desired operating mode (either 'position', 'linear_position', 'ext_position',
  ///   'velocity', 'pwm', 'current', or 'current_based_position')
  /// @param profile_type set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based
  ///   Profile (modifies Bit 2 of the 'Drive_Mode' register)
  /// @param profile_velocity passthrough to the Profile_Velocity register on the motor
  /// @param profile_acceleration passthrough to the Profile_Acceleration register on the motor
  void robot_set_operating_modes(
    const std::string cmd_type,
    const std::string & name,
    const std::string & mode,
    const std::string profile_type = DEFAULT_PROF_TYPE,
    int32_t profile_velocity = DEFAULT_PROF_VEL,
    int32_t profile_acceleration = DEFAULT_PROF_ACC);

  /// @brief Helper function used to set the operating mode for a single motor
  /// @param name desired motor name
  /// @param mode desired operating mode (either 'position', 'linear_position', 'ext_position',
  ///   'velocity', 'pwm', 'current', or 'current_based_position')
  /// @param profile_type set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based
  ///   Profile (modifies Bit 2 of the 'Drive_Mode' register)
  /// @param profile_velocity passthrough to the Profile_Velocity register on the motor
  /// @param profile_acceleration passthrough to the Profile_Acceleration register on the motor
  void robot_set_joint_operating_mode(
    const std::string & name,
    const std::string & mode,
    const std::string profile_type = DEFAULT_PROF_TYPE,
    int32_t profile_velocity = DEFAULT_PROF_VEL,
    int32_t profile_acceleration = DEFAULT_PROF_ACC);

  /// @brief Torque On/Off a specific group of motors or a single motor
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if torquing off a group of motors or
  ///   'CMD_TYPE_SINGLE' if torquing off a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param enable set to True to torque on or False to torque off
  void robot_torque_enable(
    const std::string cmd_type,
    const std::string & name,
    const bool & enable);

  /// @brief Reboot a specific group of motors or a single motor
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if rebooting a group of motors or 'CMD_TYPE_SINGLE'
  ///   if rebooting a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param torque_enable set to True to torque on or False to torque off after rebooting
  /// @param smart_reboot set to True to only reboot motor(s) in a specified group that have gone
  ///   into an error state
  void robot_reboot_motors(
    const std::string cmd_type,
    const std::string & name,
    const bool & enable,
    const bool & smart_reboot);

  /// @brief Command a desired group of motors with the specified commands
  /// @param name desired motor group name
  /// @param commands vector of commands (order matches the order specified in the 'groups'
  ///   section in the motor config file)
  /// @details commands are processed differently based on the operating mode specified for the
  ///   motor group
  void robot_write_commands(
    const std::string & name,
    std::vector<float> commands);

  /// @brief Command a desired motor with the specified command
  /// @param name desired motor name
  /// @param command motor command
  /// @details the command is processed differently based on the operating mode specified for the
  ///   motor
  void robot_write_joint_command(
    const std::string & name,
    float command);

  /// @brief Set motor firmware PID gains
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if changing the PID gains for a group of motors or
  ///   'CMD_TYPE_SINGLE' if changing the PID gains for a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param gains vector containing the desired PID gains - order is as shown in the function
  void robot_set_motor_pid_gains(
    const std::string cmd_type,
    const std::string & name,
    const std::vector<int32_t> & gains);

  /// @brief Set a register value to multiple motors
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if setting register values for a group of motors or
  ///   'CMD_TYPE_SINGLE' if setting a single register value
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param value desired register value
  void robot_set_motor_registers(
    const std::string cmd_type,
    const std::string & name,
    const std::string & reg,
    const int32_t & value);

  /// @brief Get a register value from multiple motors
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if getting register values from a group of motors or
  ///   'CMD_TYPE_SINGLE' if getting a single register value
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param values [out] vector of register values
  void robot_get_motor_registers(
    const std::string cmd_type,
    const std::string & name,
    const std::string & reg,
    std::vector<int32_t> & values);

  /// @brief Get states for a group of joints
  /// @param name desired joint group name
  /// @param positions [out] vector of current joint positions [rad]
  /// @param velocities [out] vector of current joint velocities [rad/s]
  /// @param effort [out] vector of current joint effort [mA]
  void robot_get_joint_states(
    const std::string & name,
    std::vector<float> * positions = NULL,
    std::vector<float> * velocities = NULL,
    std::vector<float> * effort = NULL);

  /// @brief Get states for a single joint
  /// @param name desired joint name
  /// @param position [out] current joint position [rad]
  /// @param velocity [out] current joint velocity [rad/s]
  /// @param effort [out] current joint effort [mA]
  void robot_get_joint_state(
    const std::string & name,
    float * position = NULL,
    float * velocity = NULL,
    float * effort = NULL);

  /// @brief Converts linear distance between two gripper fingers into angular position
  /// @param name name of the gripper servo to command
  /// @param linear_position desired distance [m] between the two gripper fingers
  /// @param <float> [out] angular position [rad] that achieves the desired linear distance
  float robot_convert_linear_position_to_radian(
    const std::string & name,
    const float & linear_position);

  /// @brief Converts angular position into the linear distance from one gripper finger to the
  ///   center of the gripper servo horn
  /// @param name name of the gripper sevo to command
  /// @param angular_position desired gripper angular position [rad]
  /// @param <float> [out] linear position [m] from a gripper finger to the center of the gripper
  ///   servo horn
  float robot_convert_angular_position_to_linear(
    const std::string & name,
    const float & angular_position);

private:
  // Frequency at which the ROS Timer publishing joint states should run
  int timer_hz;

  // Boolean that determines if joint states should be published;
  bool pub_states;

  // Boolean that changes value when a JointTrajectoryCommand begins and ends execution
  bool execute_joint_traj;

  // Pointer to the 'all' group (makes updating joint states more efficient)
  JointGroup * all_ptr;

  // DynamixelWorkbench object used to easily communicate with any Dynamixel servo
  DynamixelWorkbench dxl_wb;

  // Holds all the information in the motor_configs YAML file
  YAML::Node motor_configs;

  // Holds all the information in the mode_configs YAML file
  YAML::Node mode_configs;

  // ROS Publisher responsible for publishing joint states
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states;

  // ROS Subscriber responsible for subscribing to JointGroupCommand messages
  rclcpp::Subscription<JointGroupCommand>::SharedPtr sub_command_group;

  // ROS Subscriber responsible for subscribing to JointSingleCommand messages
  rclcpp::Subscription<JointSingleCommand>::SharedPtr sub_command_single;

  // ROS Subscriber responsible for subscribing to JointTrajectoryCommand messages
  rclcpp::Subscription<JointTrajectoryCommand>::SharedPtr sub_command_traj;

  // ROS Service Server used to set motor PID gains
  rclcpp::Service<MotorGains>::SharedPtr srv_motor_gains;

  // ROS Service Server used to set motor operating modes
  rclcpp::Service<OperatingModes>::SharedPtr srv_operating_modes;

  // ROS Service Server used to set any motor register
  rclcpp::Service<RegisterValues>::SharedPtr srv_set_registers;

  // ROS Service Server used to get any motor register
  rclcpp::Service<RegisterValues>::SharedPtr srv_get_registers;

  // ROS Service Server used to get general information about the robot (like limits)
  rclcpp::Service<RobotInfo>::SharedPtr srv_get_robot_info;

  // ROS Service Server used to torque on/off any motor
  rclcpp::Service<TorqueEnable>::SharedPtr srv_torque_enable;

  // ROS Service Server used to reboot any motor
  rclcpp::Service<Reboot>::SharedPtr srv_reboot_motors;

  // ROS Timer used to continuously publish joint states
  rclcpp::TimerBase::SharedPtr tmr_joint_states;

  // ROS One-Shot Timer used when commanding motor trajectories
  rclcpp::TimerBase::SharedPtr tmr_joint_traj;

  // Holds the most recent JointState message
  sensor_msgs::msg::JointState joint_states;

  // Holds the most recent JointTrajectoryCommand message
  JointTrajectoryCommand::SharedPtr joint_traj_cmd;

  // Indicating the trajectory start time
  rclcpp::Time traj_start_time;

  // Inficating whether to set up relative time.
  bool set_up_start_time;

  // Trajectory counter
  size_t cntr;

  // Holds the USB port name that connects to the U2D2
  std::string port = DEFAULT_PORT;

  // Desired JointState topic name
  std::string js_topic;

  // Vector containing all the desired EEPROM register values to command the motors at startup
  std::vector<MotorInfo> motor_info_vec;

  // Vector containing the order in which multiple grippers (if present) are published in the
  // JointState message
  std::vector<std::string> gripper_order;

  // Dictionary mapping register names to information about them (like 'address' and expected 'data
  // length')
  std::unordered_map<std::string, const ControlItem *> control_items;

  // Dictionary mapping a joint's name with its 'sleep' position
  std::unordered_map<std::string, float> sleep_map;

  // Dictionary mapping a joint group's name with information about it (as defined in the
  // JointGroup struct)
  std::unordered_map<std::string, JointGroup> group_map;

  // Dictionary mapping a motor's name with information about it (as defined in the MotorState
  // struct)
  std::unordered_map<std::string, MotorState> motor_map;

  // Dictionary mapping a motor's name with the names of its shadows - including itself
  std::unordered_map<std::string, std::vector<std::string>> shadow_map;

  // Dictionary mapping the name of either servo in a 2in1 motor with the other one (henceforth
  // known as 'sister')
  std::unordered_map<std::string, std::vector<std::string>> sister_map;

  // Dictionary mapping the name of a gripper motor with information about it (as defined in the
  // Gripper struct)
  std::unordered_map<std::string, Gripper> gripper_map;

  // Dictionary mapping the name of a joint with its position in the JointState 'name' list
  std::unordered_map<std::string, size_t> js_index_map;

  /// @brief Loads a robot-specific 'motor_configs' yaml file and populates class variables with
  ///   its contents
  /// @param <bool> [out] True if the motor configs were successfully retrieved; False otherwise
  bool robot_get_motor_configs(void);

  /// @brief Initializes the port to communicate with the Dynamixel servos
  /// @param <bool> [out] True if the port was successfully opened; False otherwise
  bool robot_init_port(void);

  /// @brief Pings all motors to make sure they can be found
  /// @param <bool> [out] True if all motors were found; False otherwise
  bool robot_ping_motors(void);

  /// @brief Declare all parameters needed by the node
  void robot_init_parameters(void);

  /// @brief Writes some 'startup' EEPROM register values to the Dynamixel servos
  /// @param <bool> [out] True if all register values were written successfully; False otherwise
  bool robot_load_motor_configs(void);

  /// @brief Retrieves information about 'Goal_XXX' and 'Present_XXX' registers
  /// @details Info includes a register's name, address, and data length
  void robot_init_controlItems(void);

  /// @brief Creates SyncWrite and SyncRead Handlers to write/read data to multiple motors
  ///   simultaneously
  void robot_init_workbench_handlers(void);

  /// @brief Loads a 'mode_configs' yaml file containing desired operating modes and sets up the
  ///   motors accordingly
  void robot_init_operating_modes(void);

  /// @brief Initialize ROS Publishers
  void robot_init_publishers(void);

  /// @brief Initialize ROS Subscribers
  void robot_init_subscribers(void);

  /// @brief Initialize ROS Services
  void robot_init_services(void);

  /// @brief Initialize ROS Timers
  void robot_init_timers(void);

  /// @brief Waits until first JointState message is received
  void robot_wait_for_joint_states(void);

  /// @brief ROS Subscriber callback function to command a group of joints
  /// @param msg JointGroupCommand message dictating the joint group to command along with the
  ///   actual commands
  /// @details refer to the message definition for details
  void robot_sub_command_group(const JointGroupCommand::SharedPtr msg);

  /// @brief ROS Subscriber callback function to command a single joint
  /// @param msg JointSingleCommand message dictating the joint to command along with the actual
  ///   command
  /// @details refer to the message definition for details
  void robot_sub_command_single(const JointSingleCommand::SharedPtr msg);

  /// @brief ROS Subscriber callback function to command a joint trajectory
  /// @param msg JointTrajectoryCommand message dictating the joint(s) to command along with the
  ///   desired trajectory
  /// @details refer to the message definition for details
  void robot_sub_command_traj(const JointTrajectoryCommand::SharedPtr msg);

  /// @brief ROS Service to torque the joints on the robot on/off
  /// @param req TorqueEnable service message request
  /// @param res [out] TorqueEnable service message response [unused]
  /// @details refer to the service definition for details
  bool robot_srv_torque_enable(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<TorqueEnable::Request> req,
    std::shared_ptr<TorqueEnable::Response> res);

  /// @brief ROS Service to reboot the motors on the robot
  /// @param req Reboot service message request
  /// @param res [out] Reboot service message response [unused]
  /// @details refer to the service definition for details
  bool robot_srv_reboot_motors(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<Reboot::Request> req,
    std::shared_ptr<Reboot::Response> res);

  /// @brief ROS Service that allows the user to get information about the robot
  /// @param req RobotInfo service message request
  /// @param res [out] RobotInfo service message response
  /// @details refer to the service definition for details
  bool robot_srv_get_robot_info(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<RobotInfo::Request> req,
    std::shared_ptr<RobotInfo::Response> res);

  /// @brief ROS Service that allows the user to change operating modes
  /// @param req OperatingModes service message request
  /// @param res [out] OperatingModes service message response [unused]
  /// @details refer to the service definition for details
  bool robot_srv_set_operating_modes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<OperatingModes::Request> req,
    std::shared_ptr<OperatingModes::Response> res);

  /// @brief ROS Service that allows the user to set the motor firmware PID gains
  /// @param req MotorGains service message request
  /// @param res [out] MotorGains service message response [unused]
  /// @details refer to the service defintion for details
  bool robot_srv_set_motor_pid_gains(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<MotorGains::Request> req,
    std::shared_ptr<MotorGains::Response> res);

  /// @brief ROS Service that allows the user to change a specific register to a specific value for
  ///   multiple motors
  /// @param req RegisterValues service message request
  bool robot_srv_set_motor_registers(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<RegisterValues::Request> req,
    std::shared_ptr<RegisterValues::Response> res);

  /// @brief ROS Service that allows the user to read a specific register on multiple motors
  /// @param req RegisterValues service message request
  /// @param res [out] RegisterValues service message response
  /// @details refer to the service definition for details
  bool robot_srv_get_motor_registers(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<RegisterValues::Request> req,
    std::shared_ptr<RegisterValues::Response> res);

  /// @brief Checks service call requests for validity
  /// @param cmd_type request cmd_type field
  /// @param name request name field
  /// @returns true if the service call request is valid, false otherwise
  /// @details cmd_type must be 'CMD_TYPE_SINGLE' or 'CMD_TYPE_GROUP'; name must be in the
  ///   group_map or motor_map
  bool robot_srv_validate(std::string cmd_type, std::string & name);

  /// @brief ROS One-Shot Timer used to step through a commanded joint trajectory
  void robot_execute_trajectory();

  /// @brief ROS Timer that reads current states from all the motors and publishes them to the
  ///   joint_states topic
  void robot_update_joint_states();
};

}  // namespace interbotix_xs

#endif  // INTERBOTIX_XS_SDK__XS_SDK_OBJ_HPP_
