#ifndef XS_SDK_OBJ_H_
#define XS_SDK_OBJ_H_

#include <ros/ros.h>
#include <urdf/model.h>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include "interbotix_xs_sdk/Reboot.h"
#include "interbotix_xs_sdk/RobotInfo.h"
#include "interbotix_xs_sdk/MotorGains.h"
#include "interbotix_xs_sdk/TorqueEnable.h"
#include "interbotix_xs_sdk/OperatingModes.h"
#include "interbotix_xs_sdk/RegisterValues.h"
#include "interbotix_xs_sdk/JointGroupCommand.h"
#include "interbotix_xs_sdk/JointSingleCommand.h"
#include "interbotix_xs_sdk/JointTrajectoryCommand.h"

#define BAUDRATE 1000000                                                // All motors are preset to 1M baud
#define PORT "/dev/ttyDXL"                                              // Udev rule creates a symlink with this name
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0                          // Write goal positions [rad] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1                          // Write goal velocities [rad/s] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT 2                           // Write goal currents [mA] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_PWM 3                               // Write goal pwms to multiple motors at the same time
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0       // Read current joint states for multiple motors at the same time

static const std::string OP_MODE = "position";                          // Default motor operating mode is 'position'
static const std::string PROFILE_TYPE = "velocity";                     // Default motor profile type is 'velocity' (as opposed to 'time')
static const int32_t PROFILE_VELOCITY = 0;                              // Allow joint velocity to be infinite when in position control mode - makes robot very reactive to joint commands
static const int32_t PROFILE_ACCELERATION = 0;                          // Allow joint acceleration to be infinite when in position control mode - makes robot very reactive to joint commands
static const bool TORQUE_ENABLE = true;                                 // Torque motor on by default

struct JointGroup                                                       // Struct to hold multiple joints that represent a group
{
  std::vector<std::string> joint_names;                                 // Names of all joints in the group
  std::vector<uint8_t> joint_ids;                                       // Dynamixel ID of all joints in the group
  uint8_t joint_num;                                                    // Number of joints in the group
  std::string mode;                                                     // Operating Mode for all joints in the group
  std::string profile_type;                                             // Profile Type ('velocity' or 'time') for all joints in the group
};

struct MotorState                                                       // Struct to hold data on a single motor
{
  uint8_t motor_id;                                                     // Dynamixel ID of the motor
  std::string mode;                                                     // Operating Mode of the motor
  std::string profile_type;                                             // Profile Type ('velocity' or 'time') for the motor
};

struct Gripper                                                          // Struct to hold data on an Interbotix Gripper
{
  size_t js_index;                                                      // Index in the published JointState message 'name' list belonging to the gripper motor
  float horn_radius;                                                    // Distance [m] from the motor horn's center to its edge
  float arm_length;                                                     // Distance [m] from the edge of the motor horn to a finger
  std::string left_finger;                                              // Name of the 'left_finger' joint as defined in the URDF (if present)
  std::string right_finger;                                             // Name of the 'right_finger' joint as defined in the URDF (if present)
};

struct MotorInfo                                                        // Struct to hold a desired register value for a given motor
{
  uint8_t motor_id;                                                     // Dynamixel ID of a motor
  std::string reg;                                                      // Register name
  int32_t value;                                                        // Value to write to the above register for the specified motor
};

// Interbotix Core Class to build any type of Dynamixel-based robot
class InterbotixRobotXS
{
public:

  /// @brief Constructor for the InterbotixRobotXS
  /// @param node_handle - ROS NodeHandle
  explicit InterbotixRobotXS(ros::NodeHandle *node_handle);

  /// @brief Destructor for the InterbotixRobotXS
  ~InterbotixRobotXS();

  /// @brief Set the operating mode for a specific group of motors or a single motor
  /// @param cmd_type - set to 'group' if changing the operating mode for a group of motors or 'single' if changing the operating mode for a single motor
  /// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
  /// @param mode - desired operating mode (either 'position', 'linear_position', 'ext_position', 'velocity', 'pwm', 'current', or 'current_based_position')
  /// @param profile_type - set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based Profile (modifies Bit 2 of the 'Drive_Mode' register)
  /// @param profile_velocity - passthrough to the Profile_Velocity register on the motor
  /// @param profile_acceleration - passthrough to the Profile_Acceleration register on the motor
  void robot_set_operating_modes(std::string const& cmd_type, std::string const& name, std::string const& mode, std::string const& profile_type=PROFILE_TYPE, int32_t profile_velocity=PROFILE_VELOCITY, int32_t profile_acceleration=PROFILE_ACCELERATION);

  /// @brief Helper function used to set the operating mode for a single motor
  /// @param name - desired motor name
  /// @param mode - desired operating mode (either 'position', 'linear_position', 'ext_position', 'velocity', 'pwm', 'current', or 'current_based_position')
  /// @param profile_type - set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based Profile (modifies Bit 2 of the 'Drive_Mode' register)
  /// @param profile_velocity - passthrough to the Profile_Velocity register on the motor
  /// @param profile_acceleration - passthrough to the Profile_Acceleration register on the motor
  void robot_set_joint_operating_mode(std::string const& name, std::string const& mode, std::string const& profile_type=PROFILE_TYPE, int32_t profile_velocity=PROFILE_VELOCITY, int32_t profile_acceleration=PROFILE_ACCELERATION);

  /// @brief Torque On/Off a specific group of motors or a single motor
  /// @param cmd_type - set to 'group' if torquing off a group of motors or 'single' if torquing off a single motor
  /// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
  /// @param enable - set to True to torque on or False to torque off
  void robot_torque_enable(std::string const& cmd_type, std::string const& name, bool const& enable);

  /// @brief Reboot a specific group of motors or a single motor
  /// @param cmd_type - set to 'group' if rebooting a group of motors or 'single' if rebooting a single motor
  /// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
  /// @param torque_enable - set to True to torque on or False to torque off after rebooting
  /// @param smart_reboot - set to True to only reboot motor(s) in a specified group that have gone into an error state
  void robot_reboot_motors(std::string const& cmd_type, std::string const& name, bool const& enable, bool const& smart_reboot);

  /// @brief Command a desired group of motors with the specified commands
  /// @param name - desired motor group name
  /// @param commands - vector of commands (order matches the order specified in the 'groups' section in the motor config file)
  /// @details - commands are processed differently based on the operating mode specified for the motor group
  void robot_write_commands(std::string const& name, std::vector<float> commands);

  /// @brief Command a desired motor with the specified command
  /// @param name - desired motor name
  /// @param command - motor command
  /// @details - the command is processed differently based on the operating mode specified for the motor
  void robot_write_joint_command(std::string const& name, float command);

  /// @brief Set motor firmware PID gains
  /// @param cmd_type - set to 'group' if changing the PID gains for a group of motors or 'single' if changing the PID gains for a single motor
  /// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
  /// @param gains - vector containing the desired PID gains - order is as shown in the function
  void robot_set_motor_pid_gains(std::string const& cmd_type, std::string const& name, std::vector<int32_t> const& gains);

  /// @brief Set a register value to multiple motors
  /// @param cmd_type - set to 'group' if setting register values for a group of motors or 'single' if setting a single register value
  /// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
  /// @param value - desired register value
  void robot_set_motor_registers(std::string const& cmd_type, std::string const& name, std::string const& reg, int32_t const& value);

  /// @brief Get a register value from multiple motors
  /// @param cmd_type - set to 'group' if getting register values from a group of motors or 'single' if getting a single register value
  /// @param name - desired motor group name if cmd_type is set to 'group' or the desired motor name if cmd_type is set to 'single'
  /// @param values [out] - vector of register values
  void robot_get_motor_registers(std::string const& cmd_type, std::string const& name, std::string const& reg, std::vector<int32_t> &values);

  /// @brief Get states for a group of joints
  /// @param name - desired joint group name
  /// @param positions [out] - vector of current joint positions [rad]
  /// @param velocities [out] - vector of current joint velocities [rad/s]
  /// @param effort [out] - vector of current joint effort [mA]
  void robot_get_joint_states(std::string const& name, std::vector<float> *positions=NULL, std::vector<float> *velocities=NULL, std::vector<float> *effort=NULL);

  /// @brief Get states for a single joint
  /// @param name - desired joint name
  /// @param position [out] - current joint position [rad]
  /// @param velocity [out] - current joint velocity [rad/s]
  /// @param effort [out] - current joint effort [mA]
  void robot_get_joint_state(std::string const& name, float *position=NULL, float *velocity=NULL, float *effort=NULL);

  /// @brief Converts a desired linear distance between two gripper fingers into an angular position
  /// @param name - name of the gripper servo to command
  /// @param linear_position - desired distance [m] between the two gripper fingers
  /// @param <float> [out] - angular position [rad] that achieves the desired linear distance
  float robot_convert_linear_position_to_radian(std::string const& name, float const& linear_position);

  /// @brief Converts a specified angular position into the linear distance from one gripper finger to the center of the gripper servo horn
  /// @param name - name of the gripper sevo to command
  /// @param angular_position - desired gripper angular position [rad]
  /// @param <float> [out] - linear position [m] from a gripper finger to the center of the gripper servo horn
  float robot_convert_angular_position_to_linear(std::string const& name, float const& angular_position);

private:

  int timer_hz;                                                                 // Frequency at which the ROS Timer publishing joint states should run
  bool pub_states;                                                              // Boolean that determines if joint states should be published;
  bool execute_joint_traj;                                                      // Boolean that changes value when a JointTrajectoryCommand begins and ends execution
  JointGroup *all_ptr;                                                          // Pointer to the 'all' group (makes updating joint states more efficient)
  DynamixelWorkbench dxl_wb;                                                    // DynamixelWorkbench object used to easily communicate with any Dynamixel servo
  YAML::Node motor_configs;                                                     // Holds all the information in the motor_configs YAML file
  YAML::Node mode_configs;                                                      // Holds all the information in the mode_configs YAML file

  ros::NodeHandle node;                                                         // ROS Node handle
  ros::Publisher pub_joint_states;                                              // ROS Publisher responsible for publishing joint states
  ros::Subscriber sub_command_group;                                            // ROS Subscriber responsible for subscribing to JointGroupCommand messages
  ros::Subscriber sub_command_single;                                           // ROS Subscriber responsible for subscribing to JointSingleCommand messages
  ros::Subscriber sub_command_traj;                                             // ROS Subscriber responsible for subscribing to JointTrajectoryCommand messages
  ros::ServiceServer srv_motor_gains;                                           // ROS Service Server used to set motor PID gains
  ros::ServiceServer srv_operating_modes;                                       // ROS Service Server used to set motor operating modes
  ros::ServiceServer srv_set_registers;                                         // ROS Service Server used to set any motor register
  ros::ServiceServer srv_get_registers;                                         // ROS Service Server used to get any motor register
  ros::ServiceServer srv_get_robot_info;                                        // ROS Service Server used to get general information about the robot (like limits)
  ros::ServiceServer srv_torque_enable;                                         // ROS Service Server used to torque on/off any motor
  ros::ServiceServer srv_reboot_motors;                                         // ROS Service Server used to reboot any motor
  ros::Timer tmr_joint_states;                                                  // ROS Timer used to continuously publish joint states
  ros::Timer tmr_joint_traj;                                                    // ROS One-Shot Timer used when commanding motor trajectories
  sensor_msgs::JointState joint_states;                                         // Holds the most recent JointState message
  interbotix_xs_sdk::JointTrajectoryCommand joint_traj_cmd;                     // Holds the most recent JointTrajectoryCommand message

  std::string port;                                                             // Holds the USB port name that connects to the U2D2
  std::string js_topic;                                                         // Desired JointState topic name
  std::vector<MotorInfo> motor_info_vec;                                        // Vector containing all the desired EEPROM register values to command the motors at startup
  std::vector<std::string> gripper_order;                                       // Vector containing the order in which multiple grippers (if present) are published in the JointState message
  std::unordered_map<std::string, const ControlItem*> control_items;            // Dictionary mapping register names to information about them (like 'address' and expected 'data length')
  std::unordered_map<std::string, float> sleep_map;                             // Dictionary mapping a joint's name with its 'sleep' position
  std::unordered_map<std::string, JointGroup> group_map;                        // Dictionary mapping a joint group's name with information about it (as defined in the JointGroup struct)
  std::unordered_map<std::string, MotorState> motor_map;                        // Dictionary mapping a motor's name with information about it (as defined in the MotorState struct)
  std::unordered_map<std::string, std::vector<std::string>> shadow_map;         // Dictionary mapping a motor's name with the names of its shadows - including itself
  std::unordered_map<std::string, std::vector<std::string>> sister_map;         // Dictionary mapping the name of either servo in a 2in1 motor with the other one (henceforth known as 'sister')
  std::unordered_map<std::string, Gripper> gripper_map;                         // Dictionary mapping the name of a gripper motor with information about it (as defined in the Gripper struct)
  std::unordered_map<std::string, size_t> js_index_map;                         // Dictionary mapping the name of a joint with its position in the JointState 'name' list

  /// @brief Loads a robot-specific 'motor_configs' yaml file and populates class variables with its contents
  /// @param <bool> [out] - True if the motor configs were successfully retrieved; False otherwise
  bool robot_get_motor_configs(void);

  /// @brief Initializes the port to communicate with the Dynamixel servos
  /// @param <bool> [out] - True if the port was successfully opened; False otherwise
  bool robot_init_port(void);

  /// @brief Pings all motors to make sure they can be found
  /// @param <bool> [out] - True if all motors were found; False otherwise
  bool robot_ping_motors(void);

  /// @brief Writes some 'startup' EEPROM register values to the Dynamixel servos
  /// @param <bool> [out] - True if all register values were written successfully; False otherwise
  bool robot_load_motor_configs(void);

  /// @brief Retrieves information about 'Goal_XXX' and 'Present_XXX' registers
  /// @details - Info includes a register's name, address, and data length
  void robot_init_controlItems(void);

  /// @brief Creates SyncWrite and SyncRead Handlers to write/read data to multiple motors simultaneously
  void robot_init_SDK_handlers(void);

  /// @brief Loads a 'mode_configs' yaml file containing desired operating modes and sets up the motors accordingly
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
  /// @param msg - JointGroupCommand message dictating the joint group to command along with the actual commands
  /// @details - refer to the message definition for details
  void robot_sub_command_group(const interbotix_xs_sdk::JointGroupCommand &msg);

  /// @brief ROS Subscriber callback function to command a single joint
  /// @param msg - JointSingleCommand message dictating the joint to command along with the actual command
  /// @details - refer to the message definition for details
  void robot_sub_command_single(const interbotix_xs_sdk::JointSingleCommand &msg);

  /// @brief ROS Subscriber callback function to command a joint trajectory
  /// @param msg - JointTrajectoryCommand message dictating the joint(s) to command along with the desired trajectory
  /// @details - refer to the message definition for details
  void robot_sub_command_traj(const interbotix_xs_sdk::JointTrajectoryCommand &msg);

  /// @brief ROS Service to torque the joints on the robot on/off
  /// @param req - TorqueEnable service message request
  /// @param res [out] - TorqueEnable service message response [unused]
  /// @details - refer to the service definition for details
  bool robot_srv_torque_enable(interbotix_xs_sdk::TorqueEnable::Request &req, interbotix_xs_sdk::TorqueEnable::Response &res);

  /// @brief ROS Service to reboot the motors on the robot
  /// @param req - Reboot service message request
  /// @param res [out] - Reboot service message response [unused]
  /// @details - refer to the service definition for details
  bool robot_srv_reboot_motors(interbotix_xs_sdk::Reboot::Request &req, interbotix_xs_sdk::Reboot::Response &res);

  /// @brief ROS Service that allows the user to get information about the robot
  /// @param req - RobotInfo service message request
  /// @param res [out] - RobotInfo service message response
  /// @details - refer to the service definition for details
  bool robot_srv_get_robot_info(interbotix_xs_sdk::RobotInfo::Request &req, interbotix_xs_sdk::RobotInfo::Response &res);

  /// @brief ROS Service that allows the user to change operating modes
  /// @param req - OperatingModes service message request
  /// @param res [out] - OperatingModes service message response [unused]
  /// @details - refer to the service definition for details
  bool robot_srv_set_operating_modes(interbotix_xs_sdk::OperatingModes::Request &req, interbotix_xs_sdk::OperatingModes::Response &res);

  /// @brief ROS Service that allows the user to set the motor firmware PID gains
  /// @param req - MotorGains service message request
  /// @param res [out] - MotorGains service message response [unused]
  /// @details - refer to the service defintion for details
  bool robot_srv_set_motor_pid_gains(interbotix_xs_sdk::MotorGains::Request &req, interbotix_xs_sdk::MotorGains::Response &res);

  /// @brief ROS Service that allows the user to change a specific register to a specific value for multiple motors
  /// @param req - RegisterValues service message request
  /// @param res [out] - RegisterValues service message response [unused]
  /// @details - refer to the service definition for details
  bool robot_srv_set_motor_registers(interbotix_xs_sdk::RegisterValues::Request &req, interbotix_xs_sdk::RegisterValues::Response &res);

  /// @brief ROS Service that allows the user to read a specific register on multiple motors
  /// @param req - RegisterValues service message request
  /// @param res [out] - RegisterValues service message response
  /// @details - refer to the service definition for details
  bool robot_srv_get_motor_registers(interbotix_xs_sdk::RegisterValues::Request &req, interbotix_xs_sdk::RegisterValues::Response &res);

  /// @brief ROS One-Shot Timer used to step through a commanded joint trajectory
  /// @param e - TimerEvent message [unused]
  void robot_execute_trajectory(const ros::TimerEvent &e);

  /// @brief ROS Timer that reads current states from all the motors and publishes them to the joint_states topic
  /// @param e - TimerEvent message [unused]
  void robot_update_joint_states(const ros::TimerEvent &e);
};

#endif
