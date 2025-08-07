#ifndef LIE_IMU_NODE_HPP_
#define LIE_IMU_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Dense>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// CODE BELOW ------------------------
class LieImuNode : public rclcpp::Node
{
public:
    LieImuNode();

private:
    // topic /imu/data
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_subscription_;
    void imu_data_callback_ (sensor_msgs::msg::Imu::ConstSharedPtr msg);
    rclcpp::Time last_imu_time_;

    // topic /cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    void cmd_vel_callback_ (geometry_msgs::msg::Twist::ConstSharedPtr msg);
    Eigen::Vector2d vel_;

    // EKF setup
    Eigen::Matrix4d X_; // R (phi) t (rho) 0v 1
    Matrix6d P_; // 6 = 3 from rho + 3 phi for a full twist [HOW CLEAR THE WINDSCREEN IS]
    Matrix6d Q_; // 6 same as above [WIND AND POTHOLES SO EXTERNAL FACTORS]
    // one R for each sensor
    Eigen::Matrix2d r_accel_; // 6 same as above [SENSOR/MEASUREMENT NOISE]
    // H turns state information into measurement information
    // if my state is this my measurement will probably be this
    Eigen::Matrix<double, 2, 6> H_accel_;

    // EKF functions
    void predict_ (const double dt, const Vector6d& u);
    void update_accel_ (const Eigen::Vector3d& accel);
    void update_slam_ (const Eigen::Vector3d& slam_pos, double slam_yaw);

    // SE3 lie algebra
    Eigen::Matrix3d skew_ (Eigen::Vector3d w); // gyro -> skew matrix of gyro
    Eigen::Vector3d unskew_ (Eigen::Matrix3d W); // skew matrix of gyro -> gyro
    Eigen::Matrix3d so3_exp_ (Eigen::Vector3d w); // gyro -> rotation mat
    Eigen::Vector3d so3_log_ (Eigen::Matrix3d R); // rotation mat -> gyro
    Eigen::Matrix3d so3_left_jacobian_ (Eigen::Vector3d w); 
    Eigen::Matrix3d so3_left_jacobian_inv_ (Eigen::Vector3d w);
    Eigen::Matrix4d se3_exp_ (Vector6d xi); // twist -> transformation
    Vector6d se3_log_ (Eigen::Matrix4d T); // transformation -> twist
};
// CODE ABOVE ------------------------

#endif 
