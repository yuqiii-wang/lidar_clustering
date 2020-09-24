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


nav_msgs::OccupancyGrid::ConstPtr map_ptr;
ros::Publisher filteredPtCld_pub;
float mapResolution(1.0);
unsigned int img_width(1);
unsigned int img_height(1);
float img_thres = 0.7;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  map_ptr = msg;
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  mapResolution = msg->info.resolution;
  img_width = msg->info.width;
  img_height = msg->info.height;

  std::cout << "Got map " << info.width * mapResolution;
  std::cout << "\t" << info.height * mapResolution << "\n";

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("/velodyne", "/map",  
                              ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
               pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float clipSquare = 0.15)
{
  out_cloud_ptr->points.clear();
  unsigned int cnt = 0;
  for (unsigned int ptIdx = 0; ptIdx < in_cloud_ptr->points.size(); ptIdx++)
  {
    float startClipX = in_cloud_ptr->points[ptIdx].x - clipSquare / 2;
    startClipX = startClipX > 0.0 ? startClipX : 0.0;
    float startClipY = in_cloud_ptr->points[ptIdx].y - clipSquare / 2;
    startClipY = startClipY > 0.0 ? startClipX : 0.0;
    float endClipX = in_cloud_ptr->points[ptIdx].x + clipSquare / 2;
    endClipX = endClipX < img_height ? endClipX : img_height;
    float endClipY = in_cloud_ptr->points[ptIdx].y + clipSquare / 2;
    endClipY = endClipY < img_width ? endClipX : img_width;
    for (unsigned int rowIdx = (unsigned int)(startClipX / mapResolution); 
        rowIdx < (unsigned int)((startClipX + clipSquare) / mapResolution);
        rowIdx++)
    {
      unsigned int rowPxlCnt = rowIdx * img_width;
      for (unsigned int colIdx = (unsigned int)(startClipY / mapResolution); 
        colIdx < (unsigned int)((startClipY + clipSquare) / mapResolution);
        colIdx++)
      {
        if (map_ptr->data[rowPxlCnt + colIdx] < img_thres)
        {
          out_cloud_ptr->points.push_back(in_cloud_ptr->points[ptIdx]);
        }
        else
        {
          cnt++;
          if (cnt % 100 == 0)
            std::cout << "filtered out\n";
        }
      }
    }
  }
}

void filterOutPoints(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_pclCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_pclCLoud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *in_pclCloud_ptr);

  // clipCloud(in_pclCloud_ptr, out_pclCLoud_ptr);

  in_pclCloud_ptr->header.frame_id = "velodyne";

  filteredPtCld_pub.publish(*in_pclCloud_ptr);

  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "/velodyne", "/map"));

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_pointcloud_by_map");

  ros::NodeHandle h;
  ros::NodeHandle private_nh("~");

  filteredPtCld_pub = h.advertise<sensor_msgs::PointCloud2>("filtered_pointcloud", 10);

  ros::Subscriber sub_map = h.subscribe("/map", 1, mapCallback);

  ros::Subscriber sub_ptCld = h.subscribe("/velodyne_points", 1, filterOutPoints);

  ros::spin();
  
  return 0;
}