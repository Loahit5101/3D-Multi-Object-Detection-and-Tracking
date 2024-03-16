cmake_minimum_required(VERSION 3.0.2)
project(motl)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)
find_package(GTest REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}  # Add GTest include directories
)

add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs visualization_msgs
)

add_executable(main_node scripts/main.cpp scripts/lidar_detection/lidar_detector.cpp scripts/camera_detection/camera_detector.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_main_node tests/test_main.cpp scripts/lidar_detection/lidar_detector.cpp)
target_link_libraries(test_main_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})  # Use ${GTEST_LIBRARIES} instead of ${GTEST_BOTH_LIBRARIES}
target_link_libraries(test_main_node ${GTEST_LIBRARIES} gtest_main)