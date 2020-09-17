INCLUDE(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
        std_msgs/msg/Header.proto
        common_msgs/geometry_msgs/msg/Pose.proto
        common_msgs/geometry_msgs/msg/Point.proto
        common_msgs/geometry_msgs/msg/Quaternion.proto
        common_msgs/geometry_msgs/msg/PoseArray.proto
        common_msgs/geometry_msgs/msg/SensorObstacle.proto)
ADD_LIBRARY(proto ${PROTO_HDRS} ${PROTO_SRCS})