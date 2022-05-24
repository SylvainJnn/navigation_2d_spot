cmake_minimum_required(VERSION 2.8.3)
project(navigation_2d_spot)

#########
add_compile_options(-std=c++11)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roslaunch
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  velodyne_pointcloud
  velodyne_pcl
  tf2
  std_msgs
  message_generation
  rospy
  actionlib
  actionlib_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_planning_system
)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find_package(navigation_2d_spot REQUIRED) # HERE
# include_directories(${navigation_2d_spot_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS})
include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   CreatePath.srv
   DynamicParamsDouble.srv

 )

 add_action_files(
   DIRECTORY action
   FILES
   NavigateToGoal.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   actionlib_msgs
 )



## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS
   scripts/replan.py 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )



catkin_package(
  INCLUDE_DIRS include
#    CATKIN_DEPENDS roscpp
#                   pcl_conversions
#                   pcl_ros
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)



## Declare a C++ executable
add_executable(simplified_velodyne src/simplified_velodyne.cpp)
add_executable(rpnavigate src/RPNavigate.cpp)

## Add cmake target dependencies of the executable
add_dependencies(simplified_velodyne ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(simplified_velodyne
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(rpnavigate ${catkin_LIBRARIES})
add_dependencies(rpnavigate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_executable(${PROJECT_NAME}_node src/simple_velodyne.cpp)
#add_executable(${PROJECT_NAME}_node src/point_merger.cpp)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS simplified_velodyne rpnavigate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rpnavigate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



## Install project namespaced headers !!important to have 
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )
