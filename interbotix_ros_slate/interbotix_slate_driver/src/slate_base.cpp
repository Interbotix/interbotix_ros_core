#include "interbotix_slate_driver/slate_base.h"

namespace slate_base
{

SlateBase::SlateBase(ros::NodeHandle * node_handle)
  : node(*node_handle), cmd_vel_x_(0.0), cmd_vel_z_(0.0), cnt_(0), x_(0.0), y_(0.0), theta_(0.0),
    x_vel_(0.0), z_omega_(0.0), is_first_odom_(true), pose_{0}, right_motor_c_(0.0),
    left_motor_c_(0.0), chassis_state_(SystemState::SYS_INIT), publish_tf_(false), max_vel_x_(1.0),
    max_vel_z_(1.0), current_time_(ros::Time::now()), last_time_(ros::Time::now()),
    cmd_vel_timeout_(ros::Duration(0, CMD_TIME_OUT * 1e6))
{
  node.param<bool>("publish_tf", publish_tf_, false);
  node.param<std::string>("odom_frame_name", odom_frame_name_, "odom");
  node.param<std::string>("base_frame_name", base_frame_name_, "base_link");

  pub_odom_ = node.advertise<nav_msgs::Odometry>("odom", 1);
  pub_battery_state_ = node.advertise<sensor_msgs::BatteryState>("battery_state", 1);

  sub_cmd_vel_ = node.subscribe("cmd_vel", 1, &SlateBase::cmd_vel_callback, this);

  srv_info_ = node.advertiseService(
    "set_text",
    &SlateBase::set_text_callback,
    this);
  srv_motor_torque_status_ = node.advertiseService(
    "set_motor_torque_status",
    &SlateBase::motor_torque_status_callback,
    this);

  std::string dev;
  if (!base_driver::chassisInit(dev)) {
    ROS_FATAL("Failed to initialize base.");
    ::exit(EXIT_FAILURE);
  }
  ROS_INFO("Initalized base at port '%s'.", dev.c_str());
  char version[32] = {0};
  if (base_driver::getVersion(version)) {
    ROS_INFO("Base version: %s", version);
  }
}

void SlateBase::update()
{
  ros::spinOnce();
  current_time_ = ros::Time::now();

  cnt_++;
  sensor_msgs::BatteryState state;
  if (cnt_ % 10 == 0) {
    state.header.stamp = current_time_;
    int percentage = 0;
    if (
      base_driver::getBatteryInfo(state.voltage, state.current, percentage) &&
      base_driver::getChassisState(chassis_state_))
    {
      if (state.current < 0) {
        int c = -state.current;
        int right_c = (c % 1000);
        int left_c = (c - right_c) / 1000;
        right_motor_c_ = right_c * 0.1f;
        left_motor_c_ = left_c * 0.1f;
        state.current = -(right_motor_c_ + left_motor_c_);
      }
      state.percentage = percentage;
      state.power_supply_status = chassis_state_;
      pub_battery_state_.publish(state);
    }
  }

  // time out velocity commands
  if (current_time_ - cmd_vel_time_last_update_ > cmd_vel_timeout_) {
    cmd_vel_x_ = 0.0f;
    cmd_vel_z_ = 0.0f;
  }

  cmd_vel_x_ = std::min(max_vel_x_, std::max(-max_vel_x_, cmd_vel_x_));
  cmd_vel_z_ = std::min(max_vel_z_, std::max(-max_vel_z_, cmd_vel_z_));

  base_driver::getChassisInfo(x_vel_, z_omega_);
  base_driver::getChassisOdom(x_, y_, theta_);
  base_driver::chassisControl(cmd_vel_x_, cmd_vel_z_);

  if (is_first_odom_) {
    pose_[0] = x_;
    pose_[1] = y_;
    pose_[2] = theta_;
    is_first_odom_ = false;
  }

  // create transform
  geometry_msgs::Quaternion odom_quat =
      tf::createQuaternionMsgFromYaw(wrap_angle(theta_ - pose_[2]));
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = odom_frame_name_;
  odom_trans.child_frame_id = base_frame_name_;

  odom_trans.transform.translation.x =
      cos(-pose_[2]) * (x_ - pose_[0]) - sin(-pose_[2]) * (y_ - pose_[1]);
  odom_trans.transform.translation.y =
      sin(-pose_[2]) * (x_ - pose_[0]) + cos(-pose_[2]) * (y_ - pose_[1]);
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  if (publish_tf_) {
    tf_broadcaster_odom_.sendTransform(odom_trans);
  }
  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time_;
  odom.header.frame_id = odom_frame_name_;

  // set position
  odom.pose.pose.position.x = odom_trans.transform.translation.x;
  odom.pose.pose.position.y = odom_trans.transform.translation.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance[0] = (chassis_state_ == SystemState::SYS_ESTOP) ? -1 : 1;

  // set velocity
  odom.child_frame_id = base_frame_name_;
  odom.twist.twist.linear.x = x_vel_;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = z_omega_;

  pub_odom_.publish(odom);
  last_time_ = current_time_;
}

void SlateBase::cmd_vel_callback(const geometry_msgs::Twist & msg)
{
  cmd_vel_x_ = msg.linear.x;
  cmd_vel_z_ = msg.angular.z;
  cmd_vel_time_last_update_ = ros::Time::now();
}

bool SlateBase::set_text_callback(
  interbotix_slate_msgs::SetString::Request & req,
  interbotix_slate_msgs::SetString::Response & res)
{
  res.success = base_driver::setText(req.data.c_str());
  if (res.success) {
    res.message = "Successfully set text to: '" + req.data + "'.";
    ROS_INFO(res.message.c_str());
  } else {
    res.message = "Failed to set text to: '" + req.data + "'.";
    ROS_ERROR(res.message.c_str());
  }
  return res.success;
}

bool SlateBase::motor_torque_status_callback(
    std_srvs::SetBool::Request & req,
    std_srvs::SetBool::Response & res)
{
  res.success = base_driver::motorCtrl(!req.data);
  std::string enabled_disabled = req.data ? "enable" : "disable";
  if (res.success) {
    res.message = "Successfully " + enabled_disabled + "d motor torque.";
    ROS_INFO(res.message.c_str());
  } else {
    res.message = "Failed to " + enabled_disabled + " motor torque.";
    ROS_ERROR(res.message.c_str());
  }
  return res.success;
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

} // namespace tile_base
