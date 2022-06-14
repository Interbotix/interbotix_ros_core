#include "interbotix_xs_sdk/xs_sdk_obj.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "xs_sdk");
    ros::NodeHandle n;
    bool success = true;
    InterbotixRobotXS bot(&n, success);
    if (success)
        ros::spin();
    else
    {
        ROS_FATAL(
            "[xs_sdk] For troubleshooting, please see "
            "'https://www.trossenrobotics.com/docs/interbotix_xsarms/troubleshooting/index.html'");
        ros::shutdown();
    }
    return 0;
}
