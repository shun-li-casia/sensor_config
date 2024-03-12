cmake_minimum_required(VERSION 2.8.3)
project(sensor_config)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    utility_tool
    camera_models
)
find_package(yaml-cpp REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

set(OpenCV_DIR /usr/local/opencv470)
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include  # export the head files
    LIBRARIES ${PROJECT_NAME}  # export the library
    CATKIN_DEPENDS roscpp utility_tool camera_models
)

add_library(${PROJECT_NAME}
    src/camera_models_kalibr.cc
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)
