#include "interbotix_xs_sdk/xs_sdk_obj.hpp"

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterbotixRobotXS>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
