#!/usr/bin/env bash

PROJECT_DIR=$(pwd)

cd ORB_SLAM3/
./build.sh
cd $PROJECT_DIR

mkdir build/
cd build/

cmake ..
