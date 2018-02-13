cmake_minimum_required(VERSION 2.8.3)
project(GazeboSimulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  rospy
  nav_msgs
  message_generation
  geometry_msgs
)

find_package( OpenCV REQUIRED )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bebop_vision
   CATKIN_DEPENDS  std_msgs message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

# Declare a cpp library
# add_library(bebop_cpp_libs
#   src/Object.cpp
# )

## Declare a cpp executable
# add_executable(multi_object_tracking src/multi_object_tracking.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(bebop_vision_node bebop_vision_generate_messages_cpp)

## Specify libraries to link a library or executable target against

# target_link_libraries(multi_object_tracking bebop_cpp_libs ${OpenCV_LIBRARIES}
#   ${catkin_LIBRARIES}
# )







