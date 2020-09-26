CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

project(obstaclePositionArray)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   sensor_msgs
   message_generation
)

add_message_files(
    FILES
    obstaclePositionArray.msg
  )

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
  )

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
)