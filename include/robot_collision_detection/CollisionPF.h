//
// Created by Joao Bimbo on 07/12/17.
//

#ifndef ROBOT_COLLISION_DETECTION_COLLISION_PF_H
#define ROBOT_COLLISION_DETECTION_COLLISION_PF_H

#include <Eigen/Core>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf_parser/urdf_parser.h>
#include <ros/ros.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>     // Post processing flags
#include <assimp/scene.h>           // Output data structure
#include <ros/package.h>
#include <string>
#include <robot_collision_detection/CollMesh.h>
#include <boost/smart_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <boost/smart_ptr.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>


class CollisionPF {
    struct Particle{
        int n;
        int p;
        float F;
        float K;
        float w;
    };
public:
    CollisionPF(){
        nh_=new ros::NodeHandle("~");
        ns_= nh_->getNamespace();
        this->init();
    }
    CollisionPF(ros::NodeHandle *nh,std::string ns){
        nh_=nh;
        ns_=ns;
        this->init();
    }

    void run();
    visualization_msgs::MarkerArray getMarkers();
    CollMesh::PointCloud::Ptr particlesToPointCloud(std::vector<CollisionPF::Particle> part);

protected:
    ros::NodeHandle* nh_;
    ros::Publisher pub_marray_,pub_poses_;
    ros::Subscriber sub_jointstate_;
    std::string ns_,description_param_,base_frame_,ee_frame_,robot_description_xml_,joint_states_topic_;
    KDL::Chain robot_chain_;
    KDL::Tree robot_tree_;
    KDL::JntArrayAcc jnt_array_;
    urdf::ModelInterfaceSharedPtr urdf_model_;
    sensor_msgs::JointState joint_state_;
    ros::Rate* rate_;
    std::vector<boost::shared_ptr<CollMesh> > meshes_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

    double freq_;

    void init();
    void loadMeshes(urdf::ModelInterfaceSharedPtr model,std::vector<urdf::LinkConstSharedPtr> &links_phys,std::vector<boost::shared_ptr<CollMesh> > &meshes);
    void setSensors();
    bool measurementModel(std::vector<CollisionPF::Particle> part, std::vector<KDL::Wrench> forces);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

};


#endif //ROBOT_COLLISION_DETECTION_COLLISION_PF_H
