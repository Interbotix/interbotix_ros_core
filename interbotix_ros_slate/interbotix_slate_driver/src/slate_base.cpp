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

#include <chrono>
#include <iostream>

#include "interbotix_slate_driver/slate_base.hpp"

namespace slate_base
{

SlateBase::SlateBase(const rclcpp::NodeOptions & options)
: trossen_slate::TrossenSlate(), rclcpp::Node("slate_base", options), cnt_(0), pose_{0},
  publish_tf_(false), is_first_odom_(0), current_time_(get_clock()->now()), tf_broadcaster_odom_(
    this), cmd_vel_time_last_update_(get_clock()->now()),
  cmd_vel_timeout_(rclcpp::Duration(std::chrono::milliseconds(CMD_TIME_OUT)))
{
  using std::placeholders::_1, std::placeholders::_2, std::placeholders::_3;

  declare_parameter<int>("update_frequency", 20);
  declare_parameter<bool>("publish_tf", false);
  declare_parameter<std::string>("odom_frame_name", "odom");
  declare_parameter<std::string>("base_frame_name", "base_link");

  get_parameter("update_frequency", frequency_);
  get_parameter("publish_tf", publish_tf_);
  get_parameter("odom_frame_name", odom_frame_name_);
  get_parameter("base_frame_name", base_frame_name_);

  pub_odom_ = create_publisher<Odometry>("odom", 50);
  pub_battery_state_ = create_publisher<BatteryState>("battery_state", 1);

  sub_cmd_vel_ = create_subscription<Twist>(
    "cmd_vel",
    1,
    std::bind(&SlateBase::cmd_vel_callback, this, _1));

  srv_set_text_ = create_service<SetString>(
    "set_text",
    std::bind(&SlateBase::set_text_callback, this, _1, _2, _3));

  srv_motor_torque_status_ = create_service<SetBool>(
    "set_motor_torque_status",
    std::bind(&SlateBase::motor_torque_status_callback, this, _1, _2, _3));

  srv_enable_charging_ = create_service<SetBool>(
    "enable_charging",
    std::bind(&SlateBase::enable_charging_callback, this, _1, _2, _3));

  srv_set_light_state_ = create_service<SetLightState>(
    "set_light_state",
    std::bind(&SlateBase::set_light_state_callback, this, _1, _2, _3));

  auto duration = std::chrono::milliseconds(1000 / frequency_);
  timer_ = create_wall_timer(duration, std::bind(&SlateBase::update, this));

  std::string result;
  if (!init_base(result)) {
    RCLCPP_FATAL(get_logger(), result.c_str());
  } else {
    RCLCPP_INFO(get_logger(), result.c_str());
  }
}

void SlateBase::update()
{
  current_time_ = get_clock()->now();

  // Time out velocity commands
  if (current_time_ - cmd_vel_time_last_update_ > cmd_vel_timeout_) {
    data_.cmd_vel_x = 0.0f;
    data_.cmd_vel_z = 0.0f;
  }

  // Update base state
  if (!base_driver::updateChassisInfo(&data_)) {
    return;
  }

  // Update battery state every 10 iterations
  cnt_++;
  auto battery_state = BatteryState();
  if (cnt_ % 10 == 0) {
    battery_state.header.stamp = current_time_;
    battery_state.voltage = data_.voltage;
    battery_state.temperature = std::numeric_limits<double>::quiet_NaN();
    battery_state.current = data_.current;
    battery_state.charge = std::numeric_limits<double>::quiet_NaN();
    battery_state.capacity = std::numeric_limits<double>::quiet_NaN();
    battery_state.design_capacity = std::numeric_limits<double>::quiet_NaN();
    battery_state.percentage = data_.charge;
    battery_state.present = true;
    pub_battery_state_->publish(battery_state);
  }

  // Set initial pose
  if (is_first_odom_) {
    pose_[0] = data_.odom_x;
    pose_[1] = data_.odom_y;
    pose_[2] = data_.odom_z;
    is_first_odom_ = false;
  }

  // Create transform
  tf2::Quaternion q;
  q.setRPY(0, 0, wrap_angle(data_.odom_z - pose_[2]));
  auto odom_quat = tf2::toMsg(q);
  auto odom_trans = TransformStamped();
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = odom_frame_name_;
  odom_trans.child_frame_id = base_frame_name_;

  odom_trans.transform.translation.x =
    cos(-pose_[2]) * (data_.odom_x - pose_[0]) - sin(-pose_[2]) * (data_.odom_y - pose_[1]);
  odom_trans.transform.translation.y =
    sin(-pose_[2]) * (data_.odom_x - pose_[0]) + cos(-pose_[2]) * (data_.odom_y - -pose_[1]);
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // Send the transform
  if (publish_tf_) {
    tf_broadcaster_odom_.sendTransform(odom_trans);
  }

  // Publish odometry
  auto odom = Odometry();
  odom.header.stamp = current_time_;
  odom.header.frame_id = odom_frame_name_;

  // Set pose
  odom.pose.pose.position.x = odom_trans.transform.translation.x;
  odom.pose.pose.position.y = odom_trans.transform.translation.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance[0] = (data_.system_state == SystemState::SYS_ESTOP) ? -1 : 1;

  // Set cmd velocity
  odom.child_frame_id = base_frame_name_;
  odom.twist.twist.linear.x = data_.vel_x;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = data_.vel_z;

  pub_odom_->publish(odom);
}

void SlateBase::cmd_vel_callback(const Twist::SharedPtr msg)
{
  data_.cmd_vel_x = msg->linear.x;
  data_.cmd_vel_z = msg->angular.z;
  cmd_vel_time_last_update_ = get_clock()->now();
}

bool SlateBase::set_text_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetString::Request> req,
  const std::shared_ptr<SetString::Response> res)
{
  res->success = set_text(req->data);
  res->message = "Set base screen text to: '" + req->data + "'.";
  RCLCPP_INFO(get_logger(), res->message.c_str());
  return true;
}

bool SlateBase::motor_torque_status_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetBool::Request> req,
  const std::shared_ptr<SetBool::Response> res)
{
  std::string result;
  res->success = enable_motor_torque(req->data, result);
  res->message = result;
  RCLCPP_INFO(get_logger(), res->message.c_str());
  return res->success;
}

bool SlateBase::enable_charging_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetBool::Request> req,
  const std::shared_ptr<SetBool::Response> res)
{
  std::string result;
  res->success = enable_charging(req->data, result);
  res->message = result;
  RCLCPP_INFO(get_logger(), res->message.c_str());
  return res->success;
}

bool SlateBase::set_light_state_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetLightState::Request> req,
  const std::shared_ptr<SetLightState::Response> res)
{
  LightState light_state = static_cast<LightState>(req->light_state);
  res->success = set_light_state(light_state);
  res->message = "Set light state to: '" + std::to_string(req->light_state) + "'.";
  data_.light_state = req->light_state;
  RCLCPP_INFO(get_logger(), res->message.c_str());
  return true;
}

float SlateBase::wrap_angle(float angle)
{
  if (angle > M_PI) {
    angle = angle - 2.0 * M_PI;
  } else if (angle < -M_PI) {
    angle = angle + 2.0 * M_PI;
  }
  return angle;
}

}  // namespace slate_base
