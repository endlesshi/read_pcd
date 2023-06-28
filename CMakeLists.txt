cmake_minimum_required(VERSION 2.8.3)
project(read_pcd)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
#find_package(LASlib REQUIRED  src/read_pcd/3rdLibrary/LAStools/build/cmake_install.cmake)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)



# add_executable(kittiHelper src/kittiHelper.cpp)
# target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})



add_executable(readPCD src/readPCD.cpp include/ground_plane_fitting.cpp)
target_link_libraries(readPCD ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})# src/read_pcd/3rdLibrary/LAStools/LASlib/lib/libLASlib.a
