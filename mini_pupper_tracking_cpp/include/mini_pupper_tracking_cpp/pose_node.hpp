#ifndef POSE_NODE_HPP_
#define POSE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Dense>

// CODE BELOW ------------------------
class PoseNode : public rclcpp::Node
{
public:
    PoseNode();

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    void cmd_vel_callback_ (geometry_msgs::msg::Twist::ConstSharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr orientation_subscription_;
    void orientation_callback_ (geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr pose_publisher_timer_;
    void pose_publisher_callback_ ();

    rclcpp::Time last_cmd_vel_time_;

    void predict_ (double dt, double vel_x, double vel_y);
    void update_ ();

    double x_;
    double y_;
    double roll_;
    double pitch_;
    double yaw_;
};
// CODE ABOVE ------------------------

#endif 
