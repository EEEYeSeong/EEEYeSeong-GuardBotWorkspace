cmake_minimum_required(VERSION 3.10)
project(guardbot)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/ORB_SLAM3/cmake_modules)

find_package(OpenCV 4.4)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PahoMqttCpp REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/ORB_SLAM3
    ${PROJECT_SOURCE_DIR}/ORB_SLAM3/include
    ${PROJECT_SOURCE_DIR}/ORB_SLAM3/include/CameraModels
    ${PROJECT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

add_subdirectory(${PROJECT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/g2o)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME} INTERFACE
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
    ${PROJECT_SOURCE_DIR}/ORB_SLAM3/lib/libORB_SLAM3.so
    -lboost_serialization
    -lcrypto
    -lcurl

    # 난 MQTT가 밉다
    -lpaho-mqtt3as
    -lpaho-mqttpp3
)

add_executable(slam_http src/slam_http.cc)
target_link_libraries(slam_http ${PROJECT_NAME})

add_executable(slam_mqtt src/slam_mqtt.cc)
target_link_libraries(slam_mqtt ${PROJECT_NAME})

add_executable(slam_mqtt_rasp raspberry_pi/slam_mqtt.cc)
target_link_libraries(slam_mqtt_rasp ${PROJECT_NAME})
