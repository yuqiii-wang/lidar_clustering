source ~/perception_lidar/perception_lidar/devel/setup.bash

source ~/perception_lidar/perception_lidar/devel/setup.bash; 
roslaunch lidar_euclidean_cluster_detect lidar_euclidean_cluster_detect.launch;

source ~/perception_lidar/perception_lidar/devel/setup.bash; 
rosbag play -s 10 ~/perception_lidar/2020-08-17-16-29-58.bag;

rviz