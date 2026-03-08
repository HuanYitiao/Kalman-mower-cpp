#!/bin/bash
cd build
cmake .. && make
cd ..
mkdir -p data
./build/kalman-mower
python3 scripts/plot.py data/ekf_output.csv data/ukf_output.csv