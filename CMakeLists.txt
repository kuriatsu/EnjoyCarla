cmake_minimum_required(VERSION 2.8.3)
project(enjoy_carla)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  autoware_msgs
  jsk_rviz_plugins
  visualization_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   pcl_msgs#   sensor_msgs#   std_msgs
# )

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carla_helper
#CATKIN_DEPENDS pcl_conversions
#pcl_msgs
#pcl_ros roscpp
#sensor_msgs
#std_msgs
#autoware_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/carla_helper.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(add_intensity_node src/add_intensity.cpp)
target_link_libraries(add_intensity_node ${catkin_LIBRARIES})

add_executable(fake_ndt_node src/fake_ndt.cpp)
target_link_libraries(fake_ndt_node ${catkin_LIBRARIES})

add_executable(carla_waypoint_publisher_node src/carla_waypoint_publisher.cpp)
target_link_libraries(carla_waypoint_publisher_node ${catkin_LIBRARIES})

add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})

add_executable(viz_start_goal_node src/viz_start_goal.cpp)
target_link_libraries(viz_start_goal_node ${catkin_LIBRARIES})

add_executable(viz_future_motion_node src/viz_future_motion.cpp)
target_link_libraries(viz_future_motion_node ${catkin_LIBRARIES})
