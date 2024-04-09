#ifndef INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_
#define INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_

#include "geometry_msgs/Twist.h"
#include "interbotix_slate_driver/base_driver.h"
#include "interbotix_slate_driver/serial_driver.h"
#include "interbotix_slate_msgs/SetString.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "std_srvs/SetBool.h"
#include "tf/transform_broadcaster.h"

namespace slate_base
{

#define CMD_TIME_OUT 300  // ms
#define PORT "chassis"

class SlateBase
{
public:

  /**
   * @brief Constructor for the SlateBase
   * @param node_handle ROS NodeHandle
   */
  explicit SlateBase(ros::NodeHandle * node_handle);

  /// @brief Destructor for the SlateBase
  ~SlateBase() {};

  /// @brief Process velocity commands and update robot state
  void update();

private:
  // NodeHandle
  ros::NodeHandle node;

  // Linear velocity command
  float cmd_vel_x_;

  // Angular velocity command
  float cmd_vel_z_;

  // Time last velocity command was received
  ros::Time cmd_vel_time_last_update_;

  // Odometry publisher
  ros::Publisher pub_odom_;

  // BatteryState publisher
  ros::Publisher pub_battery_state_;

  // Twist command subscriber
  ros::Subscriber sub_cmd_vel_;

  // Screen info service server
  ros::ServiceServer srv_info_;

  // Motor status service server
  ros::ServiceServer srv_motor_torque_status_;

  // Name of odom frame
  std::string odom_frame_name_;

  // Name of base frame
  std::string base_frame_name_;

  // Update counter used to only update some values less frequently
  int cnt_;

  // Odometry translation in the x-direction in meters
  float x_;

  // Odometry translation in the y-direction in meters
  float y_;

  // Odometry rotation about the z-axis in radians
  float theta_;

  // Odometry forward velocity in meters per second
  float x_vel_;

  // Odometry rotational velocity about the z-axis in radians per second
  float z_omega_;

  // Whether or not we have received our first odometry update
  bool is_first_odom_;

  // Array containing x and y translation in meters and rotation in radians
  float pose_[3];

  // Current of the right motor in Amps
  float right_motor_c_;

  // Current of the left motor in Amps
  float left_motor_c_;

  // The system state of the base
  SystemState chassis_state_;

  // Whether or not to publish base_link->odom TF
  bool publish_tf_;

  // Max linear velocity in the x-direction in meters per second
  float max_vel_x_ = 1.0;

  // Max angular velocity about the z-axis in radians per second
  float max_vel_z_ = 1.0;

  // If publish_tf_ is true, this is the broadcaster used to publish the odom->base_link TF
  tf::TransformBroadcaster tf_broadcaster_odom_;

  // Time of the current update
  ros::Time current_time_;

  // Time of the last update
  ros::Time last_time_;

  // Timeout for base velocity
  ros::Duration cmd_vel_timeout_;

  /**
   * @brief Process incoming Twist command message
   * @param msg Incoming Twist command message
   */
  void cmd_vel_callback(const geometry_msgs::Twist & msg);

  /**
   * @brief Process incoming screen text service request
   * @param req Service request containing desired text
   * @param res[out] Service response containing a success indication and a message
   * @return True if service request succeeded, false otherwise
   */
  bool set_text_callback(
    interbotix_slate_msgs::SetString::Request & req,
    interbotix_slate_msgs::SetString::Response & res);

  /**
   * @brief Process incoming motor torque status service request
   * @param req Service request containing desired motor torque status
   * @param res[out] Service response containing a success indication and a message
   * @return True if service request succeeded, false otherwise
   */
  bool motor_torque_status_callback(
    std_srvs::SetBool::Request & req,
    std_srvs::SetBool::Response & res);

  /**
   * @brief Wrap angle
   * @param angle Angle to wrap in radians
   * @return Wrapped angle
   */
  float wrap_angle(float angle);
};

}  // namespace slate_base

#endif  // INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_
