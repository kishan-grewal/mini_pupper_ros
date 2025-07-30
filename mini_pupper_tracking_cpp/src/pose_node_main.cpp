#include <rclcpp/rclcpp.hpp>
#include "mini_pupper_tracking_cpp/pose_node.hpp"

// CODE BELOW ----------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseNode>());
    rclcpp::shutdown();
    return 0;
}
