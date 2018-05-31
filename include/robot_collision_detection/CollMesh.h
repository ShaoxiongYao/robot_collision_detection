//
// Created by joao on 30/05/18.
//

#ifndef ROBOT_COLLISION_DETECTION_COLLMESH_H
#define ROBOT_COLLISION_DETECTION_COLLMESH_H

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>     // Post processing flags
#include <assimp/scene.h>           // Output data structure
#include <visualization_msgs/MarkerArray.h>
#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/features/feature.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>

class CollMesh {
    typedef pcl::PointXYZINormal PType;
    typedef pcl::PointCloud<PType> PointCloud;
public:
    CollMesh(){}
    CollMesh(aiMesh* in_mesh);
    CollMesh(std::string fname, urdf::LinkConstSharedPtr link);
    visualization_msgs::Marker getMarker();
    sensor_msgs::PointCloud2 getPointCloud();
    aiMesh* getMesh();
    PointCloud::Ptr getLikelihoods(KDL::Wrench force,KDL::Wrench mask);
    std::string link_name_;
    void setPose(KDL::Frame in);

private:
    aiMesh ass_mesh_;
    KDL::Frame pose_;
    Assimp::Importer importer_;
    visualization_msgs::Marker marker_;
    PointCloud::Ptr pcloud_;
    sensor_msgs::PointCloud2 pointcloud2_;

    void init(aiMesh* in_mesh,KDL::Frame T);
    void copyMesh(aiMesh* in_mesh,aiMesh* out_mesh);
    visualization_msgs::Marker createMarker();
    PointCloud::Ptr createPointCloud(aiMesh* in_mesh,std::string name);

};


#endif //ROBOT_COLLISION_DETECTION_COLLMESH_H
