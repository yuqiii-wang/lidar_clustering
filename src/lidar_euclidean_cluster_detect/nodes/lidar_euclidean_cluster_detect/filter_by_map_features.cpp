#include <ros/ros.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <tf/transform_broadcaster.h>



namespace filter_by_map_features
{

    void filterOutPoints(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_pclCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr out_pclCLoud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *in_pclCloud_ptr);

        in_pclCloud_ptr->header = msg->header;;

        filteredPtCld_pub.publish(*in_pclCloud_ptr);

        tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "/velodyne_points", "/map"));
        std::cout << "br sent\n";
    }
}
