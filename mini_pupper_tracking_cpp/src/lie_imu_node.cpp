#include "mini_pupper_tracking_cpp/lie_imu_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

// CODE BELOW -------------------
LieImuNode::LieImuNode()
: Node("lie_imu_node")
{
    RCLCPP_INFO(this->get_logger(), "LieImuNode has started.");

    
}

// (void)msg
// INFO_STREAM

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

void LieImuNode::test_ ()
{
    Vector6d xi;
    xi << 0.1, -0.2, 0.05, 0.01, 0.02, 0.03;

    Eigen::Matrix4d T = se3_exp_(xi);
    Vector6d xi_recovered = se3_log_(T);

    std::cout << "Original xi:      " << xi.transpose() << std::endl;
    std::cout << "Recovered xi:     " << xi_recovered.transpose() << std::endl;
    std::cout << "Difference:       " << (xi - xi_recovered).transpose() << std::endl;
}

