#include "interbotix_xs_sdk/xs_sdk_obj.hpp"

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    bool success = true;
    auto node = std::make_shared<InterbotixRobotXS>(success);
    if (success)
    {
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(node);
        exec.spin();
    }
    else
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "For troubleshooting, please see "
            "https://www.trossenrobotics.com/docs/interbotix_xsarms/troubleshooting/index.html");
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
