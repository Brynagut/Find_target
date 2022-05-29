cmake_minimum_required(VERSION 3.0.2)
project(find_target)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2_ros
  message_generation
)


find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(OpenCV 3 REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   target_position.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 std_msgs
                 cv_bridge
                 image_transport
                 tf2_ros
                 message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  
  
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(find_target src/find_target.cpp)


add_dependencies(find_target opencv_services_generate_messages_cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(find_target ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})


