#include <rclcpp/rclcpp.hpp>
#include "mini_pupper_tracking_cpp/lie_imu_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LieImuNode>());
    rclcpp::shutdown();
    return 0;
}

