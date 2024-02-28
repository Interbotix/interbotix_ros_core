#include "interbotix_slate_driver/slate_base.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tile_base");
  ros::NodeHandle node = ros::NodeHandle();
  auto driver = slate_base::SlateBase(&node);

  auto r = ros::Rate(20);
  while (ros::ok()) {
    ros::spinOnce();
    driver.update();
    r.sleep();
  }
}
