#include "ekf.h"
#include <cmath>

EKF::EKF()
{
    x = Eigen::Vector4d::Zero();
    P = Eigen::Matrix4d::Identity();
    Q = Eigen::Matrix4d::Identity();

    Q(0, 0) = 0.0001;  // x
    Q(1, 1) = 0.0001;  // y
    Q(2, 2) = 0.0001;  // yaw
    Q(3, 3) = 0.0001;  // v

    R = Eigen::Matrix2d::Identity();

    R(0, 0) = 0.03 * 0.03;  // GPS
    R(1, 1) = 0.03 * 0.03;
}

void EKF::predict(double ax, double omega, double dt)
{
    double yaw = x(2);

    x(3) += ax * dt;  // v

    double v = x(3);
    x(0) += v * std::cos(yaw) * dt;  // x
    x(1) += v * std::sin(yaw) * dt;  // y
    x(2) += omega * dt;              // yaw

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2)           = -v * std::sin(yaw) * dt;
    F(0, 3)           = std::cos(yaw) * dt;
    F(1, 2)           = v * std::cos(yaw) * dt;
    F(1, 3)           = std::sin(yaw) * dt;

    P = F * P * F.transpose() + Q;
}

void EKF::update(double gpsX, double gpsY)
{
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0)                       = 1;
    H(1, 1)                       = 1;

    Eigen::Vector2d z(gpsX, gpsY);
    Eigen::Vector2d y = z - H * x;

    Eigen::Matrix2d             S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();

    x = x + K * y;

    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P                 = (I - K * H) * P;
}