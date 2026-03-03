# Kalman-Mower Project - Progress Summary

### Project Setup
- Created GitHub repo `kalman-mower-cpp`
- Set up CMake project with Eigen3 dependency, C++17 standard

### Data Generator (Complete)
- Implemented `DataGenerator` class that simulates a 2D mower trajectory
- Motion model: constant-velocity model
  - `x[k] = x[k-1] + v * cos(yaw) * dt`
  - `y[k] = y[k-1] + v * sin(yaw) * dt`
  - `yaw[k] = yaw[k-1] + omega * dt`
- IMU simulation: acceleration + gyro with bias random walk and white noise
- GPS simulation: true position + Gaussian noise (σ = 0.3m)
- Output: CSV file with ground truth, IMU data, and GPS data

### Visualization
- Python script (`scripts/plot.py`) using matplotlib
- Plots ground truth trajectory and GPS scatter overlay
- Verified: L-shaped trajectory looks correct, GPS noise level reasonable

## Project Structure
```
kalman-mower/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── src/
│   ├── main.cpp
│   ├── data_generator.h
│   └── data_generator.cpp
├── scripts/
│   └── plot.py
└── data/
```

## Next Steps
1. Implement EKF (predict with Jacobian, update with GPS)
2. Implement UKF (sigma points, same model)
3. Add GPS dropout simulation
4. Comparison plots: EKF vs UKF vs pure GPS

## Design Decisions
- GPS noise σ=0.3m for demo purposes
- Simulation data only, no external datasets
- C++ with Eigen only, no ROS or other frameworks