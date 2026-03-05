cmake_minimum_required(VERSION 3.15)
project(armor)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(Ceres 2.0.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenVINO_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp      
            ${PROJECT_SOURCE_DIR}/Buff/buff_detector/Buff_Detector.cpp
            ${PROJECT_SOURCE_DIR}/Buff/buff_detector/Buff_Detector.h
            ${PROJECT_SOURCE_DIR}/Buff/buff_detector/inference_api2.cpp
            ${PROJECT_SOURCE_DIR}/Buff/buff_detector/inference_api2.h
            ${PROJECT_SOURCE_DIR}/Buff/buff_processor/filter/buff_predictor.cpp
            ${PROJECT_SOURCE_DIR}/Buff/buff_processor/filter/buff_predictor.h
            ${PROJECT_SOURCE_DIR}/Buff/buff_processor/filter/linear_particle_filter.cpp
            ${PROJECT_SOURCE_DIR}/Buff/buff_processor/filter/linear_particle_filter.h
            ${PROJECT_SOURCE_DIR}/Buff/buff_processor/Buff_Coordsolver.cpp
            ${PROJECT_SOURCE_DIR}/Buff/buff_processor/Buff_Coordsolver.h
            
            ${PROJECT_SOURCE_DIR}/serial/CRC_Check.cpp
            ${PROJECT_SOURCE_DIR}/serial/CRC_Check.h
            ${PROJECT_SOURCE_DIR}/serial/data_transform.cpp
            ${PROJECT_SOURCE_DIR}/serial/data_transform.h
            ${PROJECT_SOURCE_DIR}/serial/serialport.cpp
            ${PROJECT_SOURCE_DIR}/serial/serialport.h
            ${PROJECT_SOURCE_DIR}/Buff/buff_processor/Buff_Processor.cpp
            ${PROJECT_SOURCE_DIR}/Buff/buff_processor/Buff_Processor.h)

target_link_libraries(
    ${PROJECT_NAME}
    openvino::runtime
    glog
    yaml-cpp
    ${OpenCV_LIBRARIES}
    Threads::Threads
    tinyxml2::tinyxml2
    ${CERES_LIBRARIES}
)


