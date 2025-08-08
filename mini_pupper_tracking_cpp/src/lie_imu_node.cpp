#include "mini_pupper_tracking_cpp/lie_imu_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// CODE BELOW -------------------
LieImuNode::LieImuNode()
: Node("lie_imu_node")
{
    RCLCPP_INFO(this->get_logger(), "LieImuNode has started.");

    last_ekf_time_ = this->now();
    ekf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50 Hz EKF
        std::bind(&LieImuNode::ekf_loop_, this)
    );

    slam_data_fresh_ = false;

    imu_data_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&LieImuNode::imu_data_callback_, this, std::placeholders::_1)
    );

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&LieImuNode::cmd_vel_callback_, this, std::placeholders::_1)
    );

    // Add after other subscriptions:
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    slam_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LieImuNode::get_slam_pose_from_tf_, this)
    );

    X_ = Eigen::Matrix4d::Identity();
    P_ = Matrix6d::Identity();
    Q_ = Matrix6d::Zero();
    Q_.diagonal() << 2e-3, 2e-3, 5e-5, 5e-5, 5e-5, 5e-4;

    r_accel_ = Eigen::Matrix2d::Identity() * 1e-2;
    r_slam_ = Eigen::Matrix3d::Identity() * 1e-4;

    H_accel_ = Eigen::Matrix<double, 2, 6>::Zero(); // two rows of (x,y,z,r,p,ya)
    H_accel_(0, 3) = 1.0; // roll cares about roll
    H_accel_(1, 4) = 1.0; // pitch cares about pitch

    H_slam_ = Eigen::Matrix<double, 3, 6>::Zero(); // three rows of (x,y,z,r,p,ya)
    H_slam_(0, 0) = 1.0; // x cares about x
    H_slam_(1, 1) = 1.0; // y cares about y
    H_slam_(2, 5) = 1.0; // yaw cares about yaw
}

// (void)msg
// INFO_STREAM

void LieImuNode::ekf_loop_ ()
{
    rclcpp::Time ekf_time = this->now();
    const double dt = (ekf_time - last_ekf_time_).seconds();
    last_ekf_time_ = ekf_time;

    if (!last_imu_) return;

    Eigen::Vector3d accel(
        last_imu_->linear_acceleration.x,
        last_imu_->linear_acceleration.y,
        last_imu_->linear_acceleration.z
    );
    Eigen::Vector3d gyro(
        last_imu_->angular_velocity.x,
        last_imu_->angular_velocity.y,
        last_imu_->angular_velocity.z
    );

    double vel_x = 0.0;
    double vel_y = 0.0;
    if (last_twist_) {
        vel_x = last_twist_->linear.x;
        vel_y = last_twist_->linear.y;
    }
    
    Vector6d u;
    u << vel_x, vel_y, 0.0, gyro(0), gyro(1), gyro(2);
    
    // ekf
    predict_(dt, u);
    update_accel_(accel);

    if (slam_data_fresh_ && last_slam_) {
        Eigen::Vector3d slam_pos(
            last_slam_->transform.translation.x,
            last_slam_->transform.translation.y,
            last_slam_->transform.translation.z
        );
        
        tf2::Quaternion q(
            last_slam_->transform.rotation.x,
            last_slam_->transform.rotation.y,
            last_slam_->transform.rotation.z,
            last_slam_->transform.rotation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        update_slam_(slam_pos, yaw);
        slam_data_fresh_ = false;  // mark as used
    }

    // Logging
    Eigen::Vector3d t = X_.block<3,1>(0,3);
    Eigen::Matrix3d R = X_.block<3,3>(0,0);
    Eigen::Vector3d gyro_R = so3_log_(R);
    RCLCPP_INFO(this->get_logger(), 
        "aEKF: [%.3f, %.3f, %.3f] RPY: [%.2f, %.2f, %.2f]°", 
        t(0), t(1), t(2), 
        gyro_R(0)*180/M_PI, gyro_R(1)*180/M_PI, gyro_R(2)*180/M_PI);
}

void LieImuNode::imu_data_callback_ (sensor_msgs::msg::Imu::ConstSharedPtr msg)
{   
    last_imu_ = std::const_pointer_cast<sensor_msgs::msg::Imu>(msg);
}

void LieImuNode::cmd_vel_callback_ (geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    last_twist_ = std::const_pointer_cast<geometry_msgs::msg::Twist>(msg);
}

void LieImuNode::get_slam_pose_from_tf_()
{
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        last_slam_ = std::make_shared<geometry_msgs::msg::TransformStamped>(transform);
        slam_data_fresh_ = true; // was missing so slam pose wasnt being used
    } 
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "TransformException: %s", ex.what());
    }
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

void LieImuNode::update_slam_ (const Eigen::Vector3d& slam_pos, const double slam_yaw)
{
    Eigen::Vector3d z;
    z << slam_pos(0), slam_pos(1), slam_yaw; // x y yaw

    Eigen::Matrix3d R = X_.block<3, 3>(0, 0);
    Eigen::Vector3d gyro_R = so3_log_(R);
    Eigen::Vector3d t = X_.block<3, 1>(0, 3);
    Eigen::Vector3d h_hat;
    h_hat << t(0), t(1), gyro_R(2);

    // innovation
    Eigen::Vector3d y = z - h_hat;
    // y = z - hhat

    // current innovation covariance
    Eigen::Matrix3d S = H_slam_ * P_ * H_slam_.transpose() + r_slam_;
    // {6,6}*{6,3}*{3,3} = {6,3}
    Eigen::Matrix<double, 6, 3> K = P_ * H_slam_.transpose() * S.inverse();

    // SE(3): x̂⁺ = x̂⁻ ⊞ δx instead of x̂⁺ = x̂⁻ + δx
    // x = x + ky
    X_ = X_ * se3_exp_(K * y);

    // covariance counter-update:
    Matrix6d I = Matrix6d::Identity();
    //P_ *= (I - K*H_);
    // {6,6} = {6,6} - {6,3}*{3,6}
    Matrix6d KH = K * H_slam_;
    P_ = (I - KH) * P_ * (I - KH).transpose() + K * r_slam_ * K.transpose(); // joseph
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
