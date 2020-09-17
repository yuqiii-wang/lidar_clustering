#include <vector>

#include <geometry_msgs/PolygonStamped.h> // ros's geometry

#include "SensorObstacle.pb.h"
#include "PoseArray.pb.h"
#include "Pose.pb.h"
#include "Point.pb.h"
#include "Quaternion.pb.h"

#include "cluster.h"
#include "clusterExt.h"

namespace customProtoMsgs
{
    void convertSensorObs(std::vector<ClusterPtr> clusters)
    {
        auto sensorObs = std::make_shared<custom::SensorObstacle::Obstacle>();
        
        for (auto cluster : clusters)
        {
            geometry_msgs::PolygonStamped cluster_polygon = cluster->GetPolygon();

            cluster_polygon.polygon.points[0].x;
            auto poseArray = std::make_shared<custom::geometry_msgs::PoseArray>();
            poseArray->clear_poses();
            for (auto polygonPnt : cluster_polygon.polygon.points)
            {
                auto point = std::make_shared<custom::geometry_msgs::Point>();
                point->clear_x();
                point->set_x(polygonPnt.x);
                point->clear_y();
                point->set_y(polygonPnt.y);
                point->clear_z();
                point->set_z(polygonPnt.z);
                auto quat = std::make_shared<custom::geometry_msgs::Quaternion>();
                quat->clear_x();
                quat->set_x(0.0);
                quat->clear_y();
                quat->set_y(0.0);
                quat->clear_z();
                quat->set_z(0.0);
                quat->clear_w();
                quat->set_w(0.0);
                auto pose = std::make_shared<custom::geometry_msgs::Pose>();
                // pose->set_allocated_position(point);
                // pose->set_allocated_orientation(quat);

                // poseArray->add_poses();
            }
        }
    }
}