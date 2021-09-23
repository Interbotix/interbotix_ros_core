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
        ROS_ERROR("[xs_sdk] For troubleshooting, please see https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_sdk/TROUBLESHOOTING.md");
        ros::shutdown();
        
    }
    return 0;
}
