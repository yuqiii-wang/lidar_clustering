/*
    cluster extension util package, added by yuqi
    provides additional segmentation and clustering methods
*/

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/segmentation/region_growing.h>

namespace customClusteringExt
{

    int clusterByNormalAndDist(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        // const pcl::PointIndices& seg_idx,
                        std::vector<pcl::PointIndices>& out_cluster_indices
                        );

    int clusterByNormalAndDistForLargeSegs(
                        const pcl::PointCloud<pcl::PointNormal>::Ptr in_cloud_ptr,
                        const std::vector<int>& seg_idx,
                        std::vector<pcl::PointIndices>& out_cluster_indices);

    bool customRegionGrowing (
                    const pcl::PointNormal& point_a, 
                    const pcl::PointNormal& point_b,
                    float squared_distance
                    );
}