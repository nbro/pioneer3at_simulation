cmake_minimum_required(VERSION 2.8.3)
project(pioneer3at_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  nodelet
  angles
  nav_msgs tf
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  geometry_msgs)

#######################
## Gazebo Standalone ##
#######################

find_package(gazebo REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex thread)

find_package(Protobuf REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Power.msg
  ContactState.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES p2os_urdf
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(gazebo_ros_skid_steer_drive_pioneer3at src/gazebo_ros_skid_steer_drive_pioneer3at.cpp)
target_link_libraries(gazebo_ros_skid_steer_drive_pioneer3at ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazeboRosPioneer3ATContacts src/GazeboRosPioneer3ATContacts.cpp)
target_link_libraries(gazeboRosPioneer3ATContacts ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazeboRosPioneer3ATContacts ${${PROJECT_NAME}_EXPORTED_TARGETS})


## Declare a cpp executable

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  urdf
  launch
  meshes
  models
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
