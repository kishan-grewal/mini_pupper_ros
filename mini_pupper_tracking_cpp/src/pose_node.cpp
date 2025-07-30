#include "mini_pupper_tracking_cpp/pose_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// CODE BELOW -------------------
PoseNode::PoseNode()
: Node("pose_node")
{
    RCLCPP_INFO(this->get_logger(), "PoseNode has started.");

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&PoseNode::cmd_vel_callback_, this, std::placeholders::_1)
    );

    orientation_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ekf/orientation", 10,
        std::bind(&PoseNode::orientation_callback_, this, std::placeholders::_1)
    );

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "ekf/pose", 10
    );

    pose_publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PoseNode::pose_publisher_callback_, this)
    );

    last_cmd_vel_time_ = this->now();

    x_ = 0.0;
    y_ = 0.0;
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;
}

void PoseNode::cmd_vel_callback_ (geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    rclcpp::Time current_time = this->now();
    double dt = (last_cmd_vel_time_ - current_time).seconds();
    last_cmd_vel_time_ = current_time;

    double vel_x = msg->linear.x;
    double vel_y = msg->linear.y;

    predict_(dt, vel_x, vel_y);
}

void PoseNode::orientation_callback_ (geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    const geometry_msgs::msg::Quaternion& q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    RCLCPP_INFO_STREAM(this->get_logger(), std::fixed << std::setprecision(2)
    << x_ << ", " << y_ << "| " << roll_ << ", " << pitch_ << ", " << yaw_);
}

void PoseNode::pose_publisher_callback_ ()
{
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;
    
}

void PoseNode::predict_ (double dt, double vel_x, double vel_y)
{
    x_ += (vel_x * cos(yaw_) - vel_y * sin(yaw_)) * dt;
    y_ += (vel_x * sin(yaw_) + vel_y * cos(yaw_)) * dt; 
}