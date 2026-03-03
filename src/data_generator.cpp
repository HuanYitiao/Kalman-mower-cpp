#include "data_generator.h"

DataGenerator::DataGenerator(double dt) : dt(dt), rng(42)  // debug
{
    gpsNoiseStd     = 0.3;   // m
    imuAccNoiseStd  = 0.1;   // m/s^2
    imuGyroNoiseStd = 0.01;  // rad/s
    imuAccBiasStd   = 0.01;
    imuGyroBiasStd  = 0.001;
}

void DataGenerator::generate()
{
    generateTrajectory();

    for (int i = 0; i < truth.size(); i++)
    {
        imuData.push_back(simulateIMU(i));
        gpsData.push_back(simulateGPS(i));
    }
}

void DataGenerator::generateTrajectory()
{
    std::vector<Command> commands = { { 1.0, 0.0, 5.0 }, { 0.0, M_PI / 6, 3.0 }, { 1.0, 0.0, 5.0 } };

    double x   = 0;
    double y   = 0;
    double yaw = 0;

    truth.push_back({ x, y, yaw, 0 });

    for (int i = 0; i < commands.size(); i++)
    {
        auto [v, omega, duration] = commands[i];
        int steps                 = duration / dt;
        for (int t = 0; t < steps; t++)
        {
            x += v * std::cos(yaw) * dt;
            y += v * std::sin(yaw) * dt;
            yaw += omega * dt;
            truth.push_back({ x, y, yaw, v });
        }
    }
}

IMUData DataGenerator::simulateIMU(int step)
{
    if (step == 0)
    {
        return { 0, 0, 0 };
    }

    double trueAx    = (truth[step].v - truth[step - 1].v) / dt;
    double trueOmega = (truth[step].yaw - truth[step - 1].yaw) / dt;

    accBias += dist(rng) * imuAccBiasStd * std::sqrt(dt);
    gyroBias += dist(rng) * imuGyroBiasStd * std::sqrt(dt);

    double ax    = trueAx + accBias + dist(rng) * imuAccNoiseStd;
    double omega = trueOmega + gyroBias + dist(rng) * imuGyroNoiseStd;

    return { ax, 0, omega };
}

GPSData DataGenerator::simulateGPS(int step)
{
    // double t         = step * dt;
    // bool   available = !(t > 6.0 && t < 9.0);

    // if (!available)
    // {
    //     return { 0, 0, false };
    // }

    double x = truth[step].x + dist(rng) * gpsNoiseStd;
    double y = truth[step].y + dist(rng) * gpsNoiseStd;
    return { x, y, true };
}

void DataGenerator::saveCSV(const std::string& filename)
{
    std::ofstream file(filename);
    file << "t,true_x,true_y,true_yaw,true_v,imu_ax,imu_omega,gps_x,gps_y,gps_available\n";

    for (int i = 0; i < truth.size(); i++)
    {
        file << i * dt << "," << truth[i].x << "," << truth[i].y << "," << truth[i].yaw << "," << truth[i].v << ","
             << imuData[i].ax << "," << imuData[i].omega << "," << gpsData[i].x << "," << gpsData[i].y << ","
             << gpsData[i].available << "\n";
    }

    file.close();
}