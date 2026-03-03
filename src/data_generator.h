#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <random>
#include <vector>

struct State
{
    double x;
    double y;
    double yaw;
    double v;
};

struct IMUData
{
    double ax;
    double ay;
    double omega;
};

struct GPSData
{
    double x;
    double y;
    bool   available;
};

struct Command
{
    double v;
    double omega;
    double duration;
};

class DataGenerator
{
  public:
    DataGenerator(double dt);
    void generate();
    void saveCSV(const std::string& filename);

  private:
    void generateTrajectory();

    IMUData simulateIMU(int step);
    GPSData simulateGPS(int step);

    double               dt;
    std::vector<State>   truth;
    std::vector<IMUData> imuData;
    std::vector<GPSData> gpsData;

    double gpsNoiseStd;
    double imuAccNoiseStd;
    double imuGyroNoiseStd;
    double imuAccBiasStd;
    double imuGyroBiasStd;

    std::mt19937 rng;

    double                     accBias  = 0;
    double                     gyroBias = 0;
    std::normal_distribution<> dist { 0.0, 1.0 };
};