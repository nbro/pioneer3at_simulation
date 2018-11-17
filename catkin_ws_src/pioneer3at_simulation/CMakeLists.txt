cmake_minimum_required(VERSION 2.8.3)
project(pioneer3at_simulation)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs tf gazebo_dev gazebo_msgs gazebo_ros geometry_msgs)

#######################
## Gazebo Standalone ##
#######################

find_package(gazebo REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES p2os_urdf
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(gazebo_ros_skid_steer_drive_pioneer3at src/gazebo_ros_skid_steer_drive_pioneer3at.cpp)
target_link_libraries(gazebo_ros_skid_steer_drive_pioneer3at ${catkin_LIBRARIES} ${Boost_LIBRARIES})

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
