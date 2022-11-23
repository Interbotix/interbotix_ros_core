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
            "'https://docs.trossenrobotics.com/interbotix_xsarms_docs/troubleshooting.html'");
        ros::shutdown();
    }
    return 0;
}
