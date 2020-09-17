CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

project(test_filter_by_map)

find_package(catkin REQUIRED COMPONENTS 
    pcl_ros
    roscpp
    geometry_msgs
    std_msgs
    sensor_msgs
    tf
    nodelet
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Protobuf_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(test_filter_by_map
        filter_pointclouds_by_map.cpp
)

add_dependencies(test_filter_by_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_filter_by_map ${catkin_LIBRARIES} ${OpenCV_LIBS})