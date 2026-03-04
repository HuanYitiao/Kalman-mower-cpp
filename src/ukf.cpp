#include "ukf.h"
#include <cmath>

UKF::UKF()
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

    alpha = 0.001;
    beta  = 2;
    kappa = 0;
}

void UKF::predict(double ax, double omega, double dt)
{
    int    n      = 4;
    double lambda = alpha * alpha * (n + kappa) - n;

    double wm0 = lambda / (n + lambda);
    double wc0 = wm0 + (1 - alpha * alpha + beta);
    double wi  = 1.0 / (2.0 * (n + lambda));

    Eigen::Matrix4d L = ((n + lambda) * P).llt().matrixL();

    std::vector<Eigen::Vector4d> sigma(2 * n + 1);
    sigma[0] = x;
    for (int i = 0; i < n; i++)
    {
        sigma[i + 1]     = x + L.col(i);
        sigma[i + 1 + n] = x - L.col(i);
    }

    std::vector<Eigen::Vector4d> sigmaP(2 * n + 1);
    for (int i = 0; i < 2 * n + 1; i++)
    {
        double px  = sigma[i](0);
        double py  = sigma[i](1);
        double yaw = sigma[i](2);
        double v   = sigma[i](3);

        Eigen::Vector4d sp;
        sp(0)     = px + (v + ax * dt) * std::cos(yaw) * dt;
        sp(1)     = py + (v + ax * dt) * std::sin(yaw) * dt;
        sp(2)     = yaw + omega * dt;
        sp(3)     = v + ax * dt;
        sigmaP[i] = sp;
    }

    x = wm0 * sigmaP[0];
    for (int i = 1; i < 2 * n + 1; i++)
    {
        x += wi * sigmaP[i];
    }

    P = wc0 * (sigmaP[0] - x) * (sigmaP[0] - x).transpose();
    for (int i = 1; i < 2 * n + 1; i++)
    {
        P += wi * (sigmaP[i] - x) * (sigmaP[i] - x).transpose();
    }
    P += Q;
}

void UKF::update(double gpsX, double gpsY)
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