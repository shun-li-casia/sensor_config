cmake_minimum_required(VERSION 3.5)
project(sensor_config)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge utility_tool
                                        camera_models)
find_package(yaml-cpp REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

set(OpenCV_DIR /usr/local/opencv470)
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  cv_bridge
  utility_tool
  camera_models)

add_library(${PROJECT_NAME} src/modules/camera_models_kalibr.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES}
                      ${OpenCV_LIBS})

add_executable(open_camera_to_rosmsg src/tools/open_camera_to_rosmsg.cc)
target_link_libraries(open_camera_to_rosmsg ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(save_rosimg_to_file src/tools/save_rosimg_to_file.cc)
target_link_libraries(save_rosimg_to_file ${catkin_LIBRARIES} ${OpenCV_LIBS})
