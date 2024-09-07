#!/usr/bin/env bash

PROJECT_DIR=$(pwd)

cd ORB_SLAM3/
chmod +x ./build.sh
sed -i 's/++11/++14/g' CMakeLists.txt
./build.sh
cd $PROJECT_DIR

mkdir build/
cd build/

cmake ..
