#include "interbotix_footswitch_driver/footswitch_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto footswitch_node = std::make_shared<footswitch_driver::FootSwitch>();
  rclcpp::spin(footswitch_node);

  rclcpp::shutdown();

  return 0;
}
