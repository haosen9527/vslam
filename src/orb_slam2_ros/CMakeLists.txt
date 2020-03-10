cmake_minimum_required(VERSION 2.8.3)
project(orb_slam2_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall  -O3 -march=native)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${PROJECT_SOURCE_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  message_filters
  roslib
  nav_msgs
  geometry_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost  1.54.0  	     # Minimum or EXACT version e.g. 1.36.0
  REQUIRED             					# Fail with error if Boost is not found
  COMPONENTS serialization 				# Boost libraries by their canonical name
  )
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
include/orb_slam2_ros
Thirdparty
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orb_slam2_ros
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ library
 add_library(${PROJECT_NAME}  SHARED
     src/Converter.cc
     src/Frame.cc
     src/FrameDrawer.cc
     src/Initializer.cc
     src/InitKeyFrame.cc
     src/KeyFrame.cc
     src/KeyFrameDatabase.cc
     src/LocalMapping.cc
     src/LoopClosing.cc
     src/Map.cc
     src/MapDrawer.cc
     src/MapPoint.cc
     src/Optimizer.cc
     src/ORBextractor.cc
     src/ORBmatcher.cc
     src/PnPsolver.cc
     src/Sim3Solver.cc
     src/System.cc
     src/SystemSetting.cc
     src/Tracking.cc
     src/Viewer.cc
 )
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
   ${Boost_LIBRARIES}
 )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(Mono
#    test/ros_mono.cpp
#    )
### Specify libraries to link a library or executable target against
# target_link_libraries(Mono
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
# )

add_executable(RGBD
    test/ros_rgbd.cpp
    )
## Specify libraries to link a library or executable target against
 target_link_libraries(RGBD
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )
add_executable(ORBDemo
    test/ORBDemo.cpp
    )
target_link_libraries(ORBDemo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

