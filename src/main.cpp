#include "data_generator.h"
#include "ekf.h"
#include "ukf.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct Row
{
    double t, true_x, true_y, true_yaw, true_v;
    double imu_ax, imu_omega;
    double gps_x, gps_y;
    int    gps_available;
};

std::vector<Row> readCSV(const std::string& filename)
{
    std::vector<Row> rows;
    std::ifstream    file(filename);
    std::string      line;
    std::getline(file, line);  // skip header

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string       token;
        Row               r;
        std::getline(ss, token, ',');
        r.t = std::stod(token);
        std::getline(ss, token, ',');
        r.true_x = std::stod(token);
        std::getline(ss, token, ',');
        r.true_y = std::stod(token);
        std::getline(ss, token, ',');
        r.true_yaw = std::stod(token);
        std::getline(ss, token, ',');
        r.true_v = std::stod(token);
        std::getline(ss, token, ',');
        r.imu_ax = std::stod(token);
        std::getline(ss, token, ',');
        r.imu_omega = std::stod(token);
        std::getline(ss, token, ',');
        r.gps_x = std::stod(token);
        std::getline(ss, token, ',');
        r.gps_y = std::stod(token);
        std::getline(ss, token, ',');
        r.gps_available = std::stoi(token);
        rows.push_back(r);
    }
    return rows;
}

int main()
{
    double dt = 0.01;

    DataGenerator dataGenerator(dt);
    dataGenerator.generate();
    dataGenerator.saveCSV("data/output.csv");

    auto rows = readCSV("data/output.csv");

    // EKF ------------------------------------
    EKF           ekf;
    std::ofstream out_ekf("data/ekf_output.csv");
    out_ekf << "t,true_x,true_y,true_yaw,gps_x,gps_y,est_x,est_y,est_yaw\n";

    for (int i = 1; i < rows.size(); i++)
    {
        ekf.predict(rows[i].imu_ax, rows[i].imu_omega, dt);

        if (i % 10 == 0 && rows[i].gps_available)
        {
            ekf.update(rows[i].gps_x, rows[i].gps_y);
        }

        out_ekf << rows[i].t << "," << rows[i].true_x << "," << rows[i].true_y << "," << rows[i].true_yaw << ","
                << rows[i].gps_x << "," << rows[i].gps_y << "," << ekf.x(0) << "," << ekf.x(1) << "," << ekf.x(2)
                << "\n";
    }

    out_ekf.close();

    // UKF ------------------------------------
    UKF           ukf;
    std::ofstream out_ukf("data/ukf_output.csv");
    out_ukf << "t,true_x,true_y,true_yaw,gps_x,gps_y,est_x,est_y,est_yaw\n";

    for (int i = 1; i < rows.size(); i++)
    {
        ukf.predict(rows[i].imu_ax, rows[i].imu_omega, dt);

        if (i % 10 == 0 && rows[i].gps_available)
        {
            ukf.update(rows[i].gps_x, rows[i].gps_y);
        }

        out_ukf << rows[i].t << "," << rows[i].true_x << "," << rows[i].true_y << "," << rows[i].true_yaw << ","
                << rows[i].gps_x << "," << rows[i].gps_y << "," << ukf.x(0) << "," << ukf.x(1) << "," << ukf.x(2)
                << "\n";
    }

    out_ukf.close();
    return 0;
}