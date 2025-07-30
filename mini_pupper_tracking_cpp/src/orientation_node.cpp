#include "mini_pupper_tracking_cpp/orientation_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

// CODE BELOW -------------------
OrientationNode::OrientationNode()
: Node("orientation_node")
{
    RCLCPP_INFO(this->get_logger(), "OrientationNode has started.");

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&OrientationNode::imu_callback_, this, std::placeholders::_1)
    );

    orientation_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ekf/orientation", 10
    );

    orientation_publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&OrientationNode::orientation_publisher_callback_, this)
    );

    last_imu_time_ = this->now();

    x_ = Eigen::Vector3d::Zero();
    P_ = Eigen::Matrix3d::Identity();
    Q_ = Eigen::Matrix3d{
        {1e-3, 0, 0},
        {0, 1e-3, 0},
        {0, 0, 1e-3}
    };
    R_ = Eigen::Matrix2d{
        {0.1, 0},
        {0, 0.1}
    };
    H_ = Eigen::Matrix<double, 2, 3>{
        {1, 0, 0},
        {0, 1, 0}
    };
}

// (void)msg
// INFO_STREAM
void OrientationNode::imu_callback_ (sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_imu_time_).seconds();
    last_imu_time_ = current_time;

    Eigen::Vector3d w(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );

    Eigen::Vector3d a(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );

    predict_(dt, w);
    update_ (a);
}

void OrientationNode::orientation_publisher_callback_ ()
{
    tf2::Quaternion q;
    q.setRPY(x_(0), x_(1), x_(2));  // roll, pitch, yaw

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.pose.covariance[21] = P_(0,0);  // roll variance
    msg.pose.covariance[28] = P_(1,1);  // pitch variance
    msg.pose.covariance[35] = P_(2,2);  // yaw variance

    orientation_publisher_->publish(msg);
}

void OrientationNode::predict_ (double dt, const Eigen::Vector3d& w)
{
    x_ += w*dt;
    P_ += Q_; // Pkbar = A @ Pk-1 @ Atranspose + Q, A = I
}

void OrientationNode::update_ (const Eigen::Vector3d& a)
{
    double roll = atan2(a(1), a(2)); // arctan2(y/z)
    double pitch = atan2(-a(0), sqrt(a(1)*a(1) + a(2)*a(2))); // arctan2(-x/|y,z|)
    Eigen::Vector2d z = {roll, pitch};
    Eigen::Vector2d y = z - H_*x_;
    Eigen::Matrix2d S = H_*P_*H_.transpose() + R_;
    Eigen::Matrix<double, 3, 2> K = P_*H_.transpose()*S.inverse();
    x_ += K*y;

    // covariance update:
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    //P_ *= (I - K*H_);
    Eigen::Matrix3d KH = K * H_;
    P_ = (I - KH) * P_ * (I - KH).transpose() + K * R_ * K.transpose(); // joseph
}
