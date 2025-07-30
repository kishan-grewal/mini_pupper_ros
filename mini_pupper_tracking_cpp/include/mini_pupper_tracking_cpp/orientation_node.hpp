#ifndef ORIENTATION_NODE_HPP_
#define ORIENTATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Dense>

// CODE BELOW ------------------------
class OrientationNode : public rclcpp::Node
{
public:
    OrientationNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    void imu_callback_ (sensor_msgs::msg::Imu::ConstSharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr orientation_publisher_;
    rclcpp::TimerBase::SharedPtr orientation_publisher_timer_;
    void orientation_publisher_callback_ ();

    void predict_ (double dt, const Eigen::Vector3d& w);
    void update_ (const Eigen::Vector3d& a);

    rclcpp::Time last_imu_time_;

    Eigen::Vector3d x_; // roll pitch yaw
    Eigen::Matrix3d P_; // how clear the windscreen is - self asserted confidence
    Eigen::Matrix3d Q_; // wind, potholes - world forces
    Eigen::Matrix2d R_; // noise in measurements
    Eigen::Matrix<double, 2, 3> H_;
};
// CODE ABOVE ------------------------

#endif 
