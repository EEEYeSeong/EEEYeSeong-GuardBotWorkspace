#!/usr/bin/env bash

PROJECT_DIR=$(pwd)

# Build
cd ORB_SLAM3/
chmod +x ./build.sh
./build.sh
cd $PROJECT_DIR

mkdir build/
cd build/

cmake ..
cd build/
make -j4
