#pragma once
#include <Eigen/Dense>

class UKF
{
  public:
    UKF();
    void predict(double ax, double omega, double dt);
    void update(double gpsX, double gpsY);

    Eigen::Vector4d x;  // [x, y, yaw, v]
    Eigen::Matrix4d P;
    Eigen::Matrix4d Q;
    Eigen::Matrix2d R;

    double alpha;
    double beta;
    double kappa;
};