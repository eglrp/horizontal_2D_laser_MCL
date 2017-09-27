cmake_minimum_required(VERSION 2.8.3)
project(hori2dlaser_mcl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  roslaunch
  cv_bridge
  message_generation
  pcl_conversions
  pcl_ros
  geometry_msgs
  laser_geometry
  sensor_msgs
)

#add_message_files(
#   FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  sensor_msgs
#  nav_msgs
#)

#find_package(CUDA QUIET REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
#find_package(VTK REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

set(basic_source
    src/conf.h
    src/param.h
)

include_directories(
    src
)

set(hori2dlaser_mcl_source
    hori2dlaser_mcl/src/MCL.h
    hori2dlaser_mcl/src/MCL.cpp
)

add_executable(hori2dlaser_mcl hori2dlaser_mcl/hori2dlaser_mcl.cpp ${hori2dlaser_mcl_source})
target_link_libraries(hori2dlaser_mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(IU_MCL IUSLAM_generate_messages_cpp)
