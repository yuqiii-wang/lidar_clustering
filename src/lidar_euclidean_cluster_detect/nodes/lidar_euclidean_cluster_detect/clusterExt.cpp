/*
    cluster extension util package, added by yuqi
    provides additional segmentation and clustering methods
*/

#include "clusterExt.h"

bool customClusteringExt::customRegionGrowing (
                    const pcl::PointNormal& point_a, 
                    const pcl::PointNormal& point_b,
                    float squared_distance)
{
    // Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
    // float abs_normal = std::abs (point_a_normal.dot (point_b_normal));
    float diff_x = point_a.x - point_b.x, diff_y = point_a.y - point_b.y, diff_z = point_a.z - point_b.z ;
    float sqrt_diff = std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2) + std::pow(diff_z, 2));

    if (sqrt_diff < 0.25)
    {
        return (true);
    }
    else
    {
        return (false);
    }
    
}


int customClusteringExt::clusterByNormalAndDistForLargeSegs(
                            const pcl::PointCloud<pcl::PointNormal>::Ptr in_cloud_ptr,
                            const std::vector<int>& seg_idx,
                            std::vector<pcl::PointIndices>& out_cluster_indices)
{
    std::vector<int> seg_idx_tmp = seg_idx; // by copy constructor
    std::vector<int> seg_idx_lbl = seg_idx; // by copy constructor
    std::fill(seg_idx_lbl.begin(), seg_idx_lbl.end(), 0);

    if (seg_idx_tmp.size() < 5) // 5 as min num of points to be processed
    {
        return 0;
    }

    int maxLbl = 0;
    int pit_a_cnt = 0;
    for (auto pit_a = seg_idx_tmp.begin(); pit_a != seg_idx_tmp.end(); pit_a++)
    {
        int pit_b_cnt = 0;
        for (auto pit_b = seg_idx_tmp.begin(); pit_b != seg_idx_tmp.end(); pit_b++)
        {
            if (*pit_a == *pit_b)
            {
                continue;
            }
            pcl::PointNormal p_a, p_b;
            p_a.x = in_cloud_ptr->points[*pit_a].x;
            p_a.y = in_cloud_ptr->points[*pit_a].y;
            p_a.z = in_cloud_ptr->points[*pit_a].z;
            p_b.x = in_cloud_ptr->points[*pit_b].x;
            p_b.y = in_cloud_ptr->points[*pit_b].y;
            p_b.z = in_cloud_ptr->points[*pit_b].z;

            Eigen::Map<Eigen::Vector3f> point_a_normal = in_cloud_ptr->points[*pit_a].getNormalVector3fMap();
            Eigen::Map<Eigen::Vector3f> point_b_normal = in_cloud_ptr->points[*pit_b].getNormalVector3fMap();
            float abs_normal = std::abs (point_a_normal.dot (point_b_normal));
            float diff_x = p_a.x - p_b.x, diff_y = p_a.y - p_b.y, diff_z = p_a.z - p_b.z ;
            float sqrt_diff = std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2) + std::pow(diff_z, 2));

            // std::cout << "sqrt_diff " << sqrt_diff << " abs_normal " << abs_normal << "\n";
            if (sqrt_diff < 0.2 && abs_normal > 0.95)
            {
                if (seg_idx_lbl[pit_b_cnt] == 0 && seg_idx_lbl[pit_a_cnt] == 0)
                {
                    maxLbl++;
                    seg_idx_lbl[pit_a_cnt] = maxLbl;
                    seg_idx_lbl[pit_b_cnt] = maxLbl;
                }
                else if (seg_idx_lbl[pit_a_cnt] != 0)
                {
                    seg_idx_lbl[pit_b_cnt] = seg_idx_lbl[pit_a_cnt];
                }
                else if (seg_idx_lbl[pit_b_cnt] != 0)
                {
                    seg_idx_lbl[pit_a_cnt] = seg_idx_lbl[pit_b_cnt];
                }
            }
            pit_b_cnt++;
        }
        pit_a_cnt++;
    }

    for (int idx = 1; idx != maxLbl+1; idx++)
    {
        pcl::PointIndices pt_idx;
        int lbl_cnt = 0;
        for (auto idx_lbl = seg_idx_lbl.begin(); idx_lbl != seg_idx_lbl.end(); idx_lbl++)
        {
            if (*idx_lbl == idx)
            {
                pt_idx.indices.push_back(seg_idx_tmp[lbl_cnt]);
            }
            lbl_cnt++;
        }
        if (pt_idx.indices.size() > 5)
        {
            out_cluster_indices.push_back(pt_idx);
        }
    }

    // int dead_loop_count = 0;
    // while ( seg_idx_tmp.size() >= 5 )
    // {
    //     dead_loop_count++; 
    //     if (dead_loop_count > 2000)
    //     {
    //         std::cout << "loop too long\t" << out_cluster_indices.size() << "\n";
    //         break;
    //     }

    //     auto pit_a = seg_idx_tmp.begin() + 1;
    //     auto pit_b = seg_idx_tmp.begin();
    //     pcl::PointIndices pt_idx;

    //     while (pit_a != seg_idx_tmp.end())
    //     {
    //         dead_loop_count++; 
    //         if (dead_loop_count > 2000)
    //         {
    //             break;
    //         }

    //         pcl::PointNormal p_a, p_b;
    //         p_a.x = in_cloud_ptr->points[*pit_a].x;
    //         p_a.y = in_cloud_ptr->points[*pit_a].y;
    //         p_a.z = in_cloud_ptr->points[*pit_a].z;
    //         p_b.x = in_cloud_ptr->points[*pit_b].x;
    //         p_b.y = in_cloud_ptr->points[*pit_b].y;
    //         p_b.z = in_cloud_ptr->points[*pit_b].z;

    //         Eigen::Map<Eigen::Vector3f> point_a_normal = in_cloud_ptr->points[*pit_a].getNormalVector3fMap();
    //         Eigen::Map<Eigen::Vector3f> point_b_normal = in_cloud_ptr->points[*pit_b].getNormalVector3fMap();
    //         float abs_normal = std::abs (point_a_normal.dot (point_b_normal));
    //         float diff_x = p_a.x - p_b.x, diff_y = p_a.y - p_b.y, diff_z = p_a.z - p_b.z ;
    //         float sqrt_diff = std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2) + std::pow(diff_z, 2));

    //         std::cout << "sqrt_diff " << sqrt_diff << " abs_normal " << abs_normal << "\n";
    //         if (sqrt_diff < 0.7 && abs_normal > 0.85)
    //         {
    //             pt_idx.indices.push_back(*pit_a);
    //             seg_idx_tmp.erase(pit_a);
    //             continue;
    //         }

    //         pit_a++;
    //     }

    //     pit_b++;
    //     if(pit_b == seg_idx_tmp.end())
    //     {
    //         break;
    //     }
    //     if(pit_b == pit_a)
    //     {
    //         continue;
    //     }
    //     if (pt_idx.indices.size() > 10)
    //     {
    //         out_cluster_indices.push_back(pt_idx);
    //     }
    // }
    return 0;
}

int customClusteringExt::clusterByNormalAndDist(
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                            std::vector<pcl::PointIndices>& out_cluster_indices)
{

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    std::vector <pcl::PointIndices> clusterList;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (in_cloud_ptr);
    ne.setSearchMethod (tree);
    ne.setViewPoint(std::numeric_limits<float>::max(), 
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max());
    ne.setRadiusSearch (0.2);
    ne.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (10);
    reg.setMaxClusterSize (10000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (10);
    reg.setInputCloud (in_cloud_ptr);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (2.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1);
    reg.extract (clusterList);

    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance(0.5);  //
    // ec.setMinClusterSize(5);
    // ec.setMaxClusterSize(1000);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(in_cloud_ptr);
    // ec.extract(clusterList);

    for (auto small_cluster = clusterList.begin(); small_cluster != clusterList.end(); small_cluster++)
    {

        out_cluster_indices.push_back(*small_cluster);
        
        // std::cout << "out_cluster_indices " << *small_cluster << std::endl;
    }

    // std::cout << cloud_with_normals->points[0].x << "\t" << cloud_with_normals->points[0].y << std::endl;
    std::cout << "clusterList " << clusterList.size() << std::endl;
    
    return 0;
}