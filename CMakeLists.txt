cmake_minimum_required(VERSION 3.5)
project(sensor_config)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

add_subdirectory(3rdparty/libv4l2_opencv_mat)
include_directories(${libv4l2_opencv_mat_SOURCE_DIR}/inc)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge utility_tool
                                        image_transport tf tf2 rosbag)
find_package(yaml-cpp REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加Sophus库
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
find_package(FMT REQUIRED)

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
  image_transport
  tf
  tf2
  rosbag
  )

add_library(
  ${PROJECT_NAME}
  ./src/imu_model/imu.cc
  ./src/camera_model/camera.cc
  ./src/camera_model/pinhole_camera.cc
  ./src/modules/stereo_rectifier.cc
  ./src/modules/stereo_cam_config_manager.cc
  ./src/modules/stereo_imu_config.cc
  ./src/modules/imu_driver/imu_protocol.c
  ./src/modules/imu_driver/serial_device.c
  ./src/modules/imu_driver/serial_protocol.c)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES}
                      ${OpenCV_LIBS} fmt::fmt ${Sophus_LIBRARIES} v4l2cpp)

add_executable(open_camera_to_rosmsg src/tools/open_camera_to_rosmsg.cc)
target_link_libraries(open_camera_to_rosmsg ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(split_hconcate_image src/tools/split_hconcate_image.cc)
target_link_libraries(split_hconcate_image ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(undistort_img src/tools/undistort_img.cc)
target_link_libraries(undistort_img ${PROJECT_NAME})

add_executable(undistort_stereo src/tools/undistort_stereo.cc)
target_link_libraries(undistort_stereo ${PROJECT_NAME})

add_executable(open_camera_split_imu_to_rosmsg src/tools/open_camera_split_imu_to_rosmsg.cc)
target_link_libraries(open_camera_split_imu_to_rosmsg ${PROJECT_NAME})

add_executable(open_camera_split_imu_to_rosmsg_ros_time src/tools/open_camera_split_imu_to_rosmsg_ros_time.cc)
target_link_libraries(open_camera_split_imu_to_rosmsg_ros_time ${PROJECT_NAME})

add_executable(preview_rosimage src/tools/preview_rosimage.cc)
target_link_libraries(preview_rosimage ${PROJECT_NAME})

add_executable(convert_kalibr_to_vins_param
               src/tools/convert_kalibr_to_vins_param.cc)
target_link_libraries(convert_kalibr_to_vins_param ${PROJECT_NAME})

add_executable(publish_params_tp_tf
               src/tools/publish_params_tp_tf.cc)
target_link_libraries(publish_params_tp_tf ${PROJECT_NAME})

add_executable(imu_filter
               src/tools/imu_filter.cc)
target_link_libraries(imu_filter ${PROJECT_NAME})

add_executable(fake_imu src/test/fake_imu.cc)
target_link_libraries(fake_imu ${PROJECT_NAME})

add_executable(test_extreme_ros_time src/test/test_extreme_ros_time.cc)
target_link_libraries(test_extreme_ros_time ${PROJECT_NAME})

add_executable(record_start_end_odom src/tools/record_start_end_odom.cc)
target_link_libraries(record_start_end_odom ${PROJECT_NAME})

add_executable(open_camera_split_imu_to_rosmsg_v4l2 src/tools/open_camera_split_imu_to_rosmsg_v4l2.cc)
target_link_libraries(open_camera_split_imu_to_rosmsg_v4l2 ${PROJECT_NAME})

add_executable(open_imu src/tools/open_imu.cc)
target_link_libraries(open_imu ${PROJECT_NAME})
