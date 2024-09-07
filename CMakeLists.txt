cmake_minimum_required (VERSION 3.10)
project (guardbot)

add_subdirectory(${CMAKE_SOURCE_DIR}/ORB_SLAM3)

add_executable (
    ${PROJECT_NAME} src/slam.cc
)