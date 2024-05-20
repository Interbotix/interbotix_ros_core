#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hidapi.h"
#include "interbotix_footswitch_msgs/msg/footswitch_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace footswitch_driver
{

const unsigned short vend_id = 0x3553;
const unsigned short prod_id = 0xb001;

using interbotix_footswitch_msgs::msg::FootswitchState;

class FootSwitch : public rclcpp::Node
{
public:
  explicit FootSwitch(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~FootSwitch();

private:
  rclcpp::TimerBase::SharedPtr tmr_update_state_;
  hid_device *handle_;
  rclcpp::Publisher<FootswitchState>::SharedPtr pub_footswitch_state_;
  void update_state();
};

}  // namespace footswitch_driver
