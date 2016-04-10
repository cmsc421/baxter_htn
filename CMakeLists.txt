cmake_minimum_required(VERSION 2.8.3)
project(baxter_htn)
INCLUDE(FindPkgConfig)

find_package(Boost COMPONENTS date_time filesystem system thread REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  genmsg
  baxter_core_msgs
  pcl_ros 
  pcl_conversions
  geometry_msgs
)

find_package(PCL REQUIRED QUIET COMPONENTS common)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs baxter_core_msgs message_runtime geometry_msgs
  DEPENDS system_lib 
)

include_directories(
  include
  ${roscpp_INCLUDE_DIRS} 
  ${std_msgs_INCLUDE_DIRS} 
  ${sensor_msgs_INCLUDE_DIRS} 
  ${baxter_core_msgs_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  lib
)

#add_executable(test1 src/test.cpp)
#target_link_libraries(test1
#   ${roscpp_LIBRARIES} 
#   ${std_msgs_LIBRARIES} 
#   ${catkin_LIBRARIES}
# )

add_executable(baxter_ik_service src/baxter_ik_service.cpp)
add_dependencies(baxter_ik_service baxter_ik_service baxter_ik_service_gencfg baxter_pickup_msgs_gencpp)
target_link_libraries(baxter_ik_service
   ${roscpp_LIBRARIES} 
   ${std_msgs_LIBRARIES} 
   ${sensor_msgs_LIBRARIES} 
   ${catkin_LIBRARIES}
   baxter_ik
   block_tracker
 )

add_executable(baxter_block_tracker src/baxter_block_tracker.cpp)
add_dependencies(baxter_block_tracker baxter_block_tracker_gencpp baxter_block_tracker_gencfg baxter_pickup_msgs_gencpp)
target_link_libraries(baxter_block_tracker
   ${roscpp_LIBRARIES} 
   ${std_msgs_LIBRARIES} 
   ${sensor_msgs_LIBRARIES} 
   ${catkin_LIBRARIES}
   baxter_ik
   block_tracker
 )

