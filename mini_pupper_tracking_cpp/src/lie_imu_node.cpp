#include "mini_pupper_tracking_cpp/lie_imu_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

// CODE BELOW -------------------
LieImuNode::LieImuNode()
: Node("lie_imu_node")
{
    RCLCPP_INFO(this->get_logger(), "LieImuNode has started.");

    imu_data_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&LieImuNode::imu_data_callback_, this, std::placeholders::_1)
    );
    last_imu_time_ = this->now();

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&LieImuNode::cmd_vel_callback_, this, std::placeholders::_1)
    );
    vel_ = Eigen::Vector2d::Zero();

    X_ = Eigen::Matrix4d::Identity();
    P_ = Matrix6d::Identity();
    Q_ = Matrix6d::Identity() * 1e-3;
    r_accel_ = Eigen::Matrix2d::Identity() * 1e-2;
    H_accel_ = Eigen::Matrix<double, 2, 6>::Zero();
    H_accel_(0, 3) = 1.0;
    H_accel_(1, 4) = 1.0;
}

// (void)msg
// INFO_STREAM

void LieImuNode::imu_data_callback_ (sensor_msgs::msg::Imu::ConstSharedPtr msg)
{   
    rclcpp::Time current_time(msg->header.stamp);
    const double dt = (current_time - last_imu_time_).seconds();
    last_imu_time_ = current_time;

    Eigen::Vector3d accel;
    accel << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    Eigen::Vector3d gyro;
    gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    Vector6d u;
    u << vel_(0), vel_(1), 0.0, gyro(0), gyro(1), gyro(2);

    predict_(dt, u);
    update_accel_ (accel);

    // Quick logging
    Eigen::Vector3d pos = X_.block<3,1>(0,3);
    Eigen::Matrix3d R = X_.block<3,3>(0,0);
    Eigen::Vector3d rpy = so3_log_(R);
    RCLCPP_INFO(this->get_logger(), 
        "Pos: [%.3f, %.3f, %.3f] RPY: [%.3f, %.3f, %.3f]", 
        pos(0), pos(1), pos(2), 
        rpy(0)*180/M_PI, rpy(1)*180/M_PI, rpy(2)*180/M_PI);
}

void LieImuNode::cmd_vel_callback_ (geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    vel_ << msg->linear.x, msg->linear.y;
}

void LieImuNode::predict_ (const double dt, const Vector6d& u)
{
    Vector6d xi = u * dt;
    Eigen::Matrix4d T = se3_exp_(xi);
    // world->new = world->old * old->new
    X_ = X_ * T;

    Matrix6d F = Matrix6d::Identity();
    // update covariance with continuous Q
    P_ = F * P_ * F.transpose() + Q_ * dt;
}

void LieImuNode::update_accel_ (const Eigen::Vector3d& accel) 
{
    double roll_measured = atan2(accel(1), 
                                accel(2));
    double pitch_measured = atan2(-accel(0), 
    sqrt(accel(1) * accel(1) + accel(2) * accel(2)));
    Eigen::Vector2d z;
    z << roll_measured, pitch_measured; // r then p then y

    Eigen::Matrix3d R = X_.block<3, 3>(0, 0);
    Eigen::Vector3d gyro_R = so3_log_(R);
    Eigen::Vector2d h_hat;
    h_hat << gyro_R(0), gyro_R(1); // roll and pitch

    // innovation
    Eigen::Vector2d y = z - h_hat;
    // y = z - hhat

    // current innovation covariance
    Eigen::Matrix2d S = H_accel_ * P_ * H_accel_.transpose() + r_accel_;
    // {6,6}*{6,2}*{2,2} = {6,2}
    Eigen::Matrix<double, 6, 2> K = P_ * H_accel_.transpose() * S.inverse();

    // SE(3): x̂⁺ = x̂⁻ ⊞ δx instead of x̂⁺ = x̂⁻ + δx
    // x = x + ky
    X_ = X_ * se3_exp_(K * y);

    // covariance counter-update:
    Matrix6d I = Matrix6d::Identity();
    //P_ *= (I - K*H_);
    // {6,6} = {6,6} - {6,2}*{2,6}
    Matrix6d KH = K * H_accel_;
    P_ = (I - KH) * P_ * (I - KH).transpose() + K * r_accel_ * K.transpose(); // joseph
}

Eigen::Matrix3d LieImuNode::skew_ (Eigen::Vector3d phi)
{
    Eigen::Matrix3d phi_hat;

    phi_hat << 0.0, -phi(2), phi(1),
               phi(2), 0.0, -phi(0),
               -phi(1), phi(0), 0.0;
    return phi_hat;
}
Eigen::Vector3d LieImuNode::unskew_ (Eigen::Matrix3d phi_hat)
{
    Eigen::Vector3d phi;

    phi << phi_hat(2, 1), phi_hat(0, 2), phi_hat(1, 0);
    return phi;
}
Eigen::Matrix3d LieImuNode::so3_exp_ (Eigen::Vector3d phi)
{
    Eigen::Matrix3d R;

    double theta = phi.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d phi_hat = skew_(phi);

    // linear approximation
    if (theta < 1e-8) {
        R = I + phi_hat;
    }
    else {
        R = I + std::sin(theta) / theta * phi_hat +
        (1.0 - std::cos(theta)) / (theta * theta) * phi_hat * phi_hat;
    }
    return R;
}
Eigen::Vector3d LieImuNode::so3_log_ (Eigen::Matrix3d R)
{
    Eigen::Vector3d phi;

    double cos_theta = (R.trace() - 1.0) / 2.0;
    cos_theta = std::min(1.0, std::max(-1.0, cos_theta));
    double theta = std::acos(cos_theta);
    Eigen::Matrix3d R_asym = R - R.transpose();

    // linear approximation
    if (theta < 1e-8) {
        phi = unskew_(R_asym / 2);
    }
    else {
        phi = unskew_(theta / (2 * std::sin(theta)) * R_asym);
    }
    return phi;
}
Eigen::Matrix3d LieImuNode::so3_left_jacobian_ (Eigen::Vector3d phi)
{
    Eigen::Matrix3d J;

    double theta = phi.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d phi_hat = skew_(phi);

    // linear approximation
    if (theta < 1e-8) {
        J = I + phi_hat / 2;
    }
    else {
        double theta2 = theta * theta;
        double c1 = (1 - std::cos(theta)) / theta2;
        double c2 = (theta - std::sin(theta)) / (theta2 * theta);
        J = I + c1 * phi_hat + c2 * phi_hat * phi_hat;
    }
    return J;
}
Eigen::Matrix3d LieImuNode::so3_left_jacobian_inv_ (Eigen::Vector3d phi)
{
    Eigen::Matrix3d J_inv;

    double theta = phi.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d phi_hat = skew_(phi);

    J_inv = I - phi_hat / 2;
    // linear approximation done already
    if (theta > 1e-8) {
        double c1 = 1 / (theta * theta) -
                    (1 + std::cos(theta)) / (2 * theta * std::sin(theta));
        J_inv = J_inv - c1 * phi_hat;
    }
    return J_inv;
}
Eigen::Matrix4d LieImuNode::se3_exp_ (Vector6d xi)
{
    // set 0 0 0 1
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    Eigen::Vector3d rho = xi.head<3>();
    Eigen::Vector3d phi = xi.tail<3>();
    Eigen::Matrix3d R = so3_exp_(phi);
    Eigen::Matrix3d J = so3_left_jacobian_(phi);
    Eigen::Vector3d t = J * rho;
    
    // set R t
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}
Vector6d LieImuNode::se3_log_ (Eigen::Matrix4d T)
{   
    Vector6d xi;

    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    Eigen::Vector3d phi = so3_log_(R);
    Eigen::Matrix3d J_inv = so3_left_jacobian_inv_(phi);
    Eigen::Vector3d rho = J_inv * t;
    xi.head<3>() = rho;
    xi.tail<3>() = phi;
    return xi;
}
