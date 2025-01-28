// Copyright 2025 Trossen Robotics
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

#ifndef INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_
#define INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interbotix_slate_msgs/srv/set_light_state.hpp"
#include "interbotix_slate_msgs/srv/set_string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "trossen_slate/trossen_slate.hpp"

namespace slate_base
{
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using interbotix_slate_msgs::srv::SetLightState;
using interbotix_slate_msgs::srv::SetString;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::BatteryState;
using std_srvs::srv::SetBool;

class SlateBase
  : public trossen_slate::TrossenSlate,
  public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the SlateBase
   * @param options ROS NodeOptions
   */
  explicit SlateBase(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor for SlateBase
  ~SlateBase() {}

  /// @brief Process velocity commands and update robot state
  void update();

private:
  // Update counter used to update the battery status less frequently
  int cnt_;

  // Frequency of the update timer
  int frequency_;

  // Array containing x and y translation in meters and rotation in radians
  float pose_[3];

  // Whether or not to publish TF from base_link->odom
  bool publish_tf_;

  // Whether or not we have received our first odometry update
  bool is_first_odom_;

  // Base light state - see interbotix_slate_msgs/srv/SetLightState for details
  uint32_t light_state_ = 0;

  // Base command bytes containing data about charging and motor torque enabling
  uint32_t sys_cmd_ = 0;

  // Stored data of the base
  base_driver::ChassisData data_;

  // Name of odom frame
  std::string odom_frame_name_;

  // Name of base frame
  std::string base_frame_name_;

  // Time of the current update
  rclcpp::Time current_time_;

  // Odometry publisher
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;

  // BatteryState publisher
  rclcpp::Publisher<BatteryState>::SharedPtr pub_battery_state_;

  // Twist command subscriber
  rclcpp::Subscription<Twist>::SharedPtr sub_cmd_vel_;

  // Set screen text service server
  rclcpp::Service<SetString>::SharedPtr srv_set_text_;

  // Motor status service server
  rclcpp::Service<SetBool>::SharedPtr srv_motor_torque_status_;

  // Set charge enable service server
  rclcpp::Service<SetBool>::SharedPtr srv_enable_charging_;

  // Set light state service server
  rclcpp::Service<SetLightState>::SharedPtr srv_set_light_state_;

  // Timer used to update the base state
  rclcpp::TimerBase::SharedPtr timer_;

  // If publish_tf_ is true, this is the broadcaster used to publish the odom->base_link TF
  tf2_ros::TransformBroadcaster tf_broadcaster_odom_;

  // Timeout for base velocity
  rclcpp::Duration cmd_vel_timeout_;

  // Time last velocity command was received
  rclcpp::Time cmd_vel_time_last_update_;

  /**
   * @brief Process incoming Twist command message
   * @param msg Incoming Twist command message
   */
  void cmd_vel_callback(const Twist::SharedPtr msg);

  /**
   * @brief Process incoming screen text service request
   * @param request_header Incoming RMW request identifier
   * @param req Service request containing desired text
   * @param res[out] Service response containing a success indication and a message
   * @return true if service succeeded, false otherwise
   */
  bool set_text_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SetString::Request> req,
    const std::shared_ptr<SetString::Response> res);

  /**
   * @brief Process incoming motor torque status service request
   * @param request_header Incoming RMW request identifier
   * @param req Service request containing desired motor torque status
   * @param res[out] Service response containing a success indication and a message
   * @return true if service succeeded, false otherwise
   */
  bool motor_torque_status_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SetBool::Request> req,
    const std::shared_ptr<SetBool::Response> res);

  /**
   * @brief Process incoming enable charging service request
   * @param request_header Incoming RMW request identifier
   * @param req Service request containing desired charging enable status
   * @param res[out] Service response containing a success indication and a message
   * @return true if service succeeded, false otherwise
   */
  bool enable_charging_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SetBool::Request> req,
    const std::shared_ptr<SetBool::Response> res);

  /**
   * @brief Process incoming set light state service request
   * @param request_header Incoming RMW request identifier
   * @param req Service request containing desired light state
   * @param res[out] Service response containing a success indication and a message
   * @return true if service succeeded, false otherwise
   */
  bool set_light_state_callback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<SetLightState::Request> req,
    const std::shared_ptr<SetLightState::Response> res);

  /**
   * @brief Wrap angle
   * @param angle Angle to wrap in radians
   * @return Wrapped angle
   */
  float wrap_angle(float angle);
};

}  // namespace slate_base

#endif  // INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_
