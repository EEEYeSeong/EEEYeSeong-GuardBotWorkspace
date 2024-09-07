#!/usr/bin/env bash

PROJECT_DIR=$(pwd)

# Patch
patch -d ORB_SLAM3/include -p2 --binary < patch/System.h.patch

cd ORB_SLAM3/
chmod +x ./build.sh
sed -i 's/++11/++14/g' CMakeLists.txt
./build.sh
cd $PROJECT_DIR

mkdir build/
cd build/

cmake ..
cd build/
make -j4
