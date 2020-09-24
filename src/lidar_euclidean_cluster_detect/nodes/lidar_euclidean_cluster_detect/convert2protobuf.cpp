#include <vector>

#include <geometry_msgs/PolygonStamped.h> // ros's geometry

#include "proto/SensorObstacle.pb.h"
#include "proto/PoseArray.pb.h"
#include "proto/Pose.pb.h"
#include "proto/Point.pb.h"
#include "proto/Quaternion.pb.h"

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

            custom::geometry_msgs::Pose* eachPose;
            custom::geometry_msgs::Point* eachPoint;
            custom::geometry_msgs::PoseArray* eachPolygon;
            eachPolygon = sensorObs->add_obstacle();
            for (auto polygonPnt : cluster_polygon.polygon.points)
            {
                eachPose = eachPolygon->add_poses();
                eachPoint = eachPose->mutable_position();
                eachPoint->set_x(polygonPnt.x);
                eachPoint->set_y(polygonPnt.y);
                eachPoint->set_z(polygonPnt.z);
            }
        }
    }
}