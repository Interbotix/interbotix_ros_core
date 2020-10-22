#include "interbotix_xs_sdk/xs_sdk_obj.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "xs_sdk");
    ros::NodeHandle n;
    InterbotixRobotXS bot(&n);
    ros::spin();
    return 0;
}
