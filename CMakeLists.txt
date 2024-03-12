cmake_minimum_required(VERSION 3.0.2)
project(motl)

add_compile_options(-std=c++14)
SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)

add_definitions(${PCL_DEFINITIONS})


catkin_package(

   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs visualization_msgs

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

add_executable(main_node scripts/main.cpp scripts/lidar_detection/lidar_detector.cpp scripts/camera_detection/camera_detector.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})


