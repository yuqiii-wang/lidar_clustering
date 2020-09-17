gnome-terminal -e "bash -c 'source ~/lidar_detect/devel/setup.bash; roslaunch lidar_euclidean_cluster_detect lidar_euclidean_cluster_detect.launch;'"

gnome-terminal -e "bash -c 'source ~/lidar_detect/devel/setup.bash; rosbag play -s 10 2020-08-17-16-29-58.bag;'"

gnome-terminal -e "rviz"

# gnome-terminal -e "bash -c \"ssh user@address; uname -a; exec bash\""