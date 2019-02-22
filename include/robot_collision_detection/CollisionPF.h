//
// Created by Joao Bimbo on 07/12/17.
//

#ifndef ROBOT_COLLISION_DETECTION_COLLISION_PF_H
#define ROBOT_COLLISION_DETECTION_COLLISION_PF_H

#include <vector>
#include <string>
#include <random>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <urdf_parser/urdf_parser.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>     // Post processing flags
#include <assimp/scene.h>           // Output data structure
#include <boost/smart_ptr.hpp>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <robot_collision_detection/CollMesh.h>
#include <robot_collision_detection/GetParts.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

class CollisionPF {
public:
    struct Particle{
        int n;
        int p;
        double F;
        double K;
        double w;
    };
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
    void step(std::vector<CollisionPF::Particle> &parts, std::vector<KDL::Wrench> measurements, double m_s, double m_a);
    std::vector<KDL::Frame> prev_state_;

protected:
    ros::NodeHandle* nh_;
    ros::Publisher pub_marray_,pub_poses_,pub_pc_;
    ros::Subscriber sub_jointstate_;
    ros::ServiceServer srv_parts_,srv_restart_,srv_step_,srv_pause_;
    std::string ns_,description_param_,base_frame_,ee_frame_,robot_description_xml_,joint_states_topic_;
    KDL::Chain robot_chain_;
    KDL::Tree robot_tree_;
    KDL::JntArrayAcc jnt_array_;
    urdf::ModelInterfaceSharedPtr urdf_model_;
    sensor_msgs::JointState joint_state_;
    ros::Rate* rate_;
    std::vector<boost::shared_ptr<CollMesh> > meshes_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::vector<int> sensor_types;
    std::vector<double> std_devs_;
    int num_parts_;
    bool run_;
    double freq_,e_alpha_,perc_new_;
    std::vector<CollisionPF::Particle> parts;
    std::vector<CollisionPF::Particle> part_ranges;
    void init();
    bool loadParameters();
    void loadMeshes(urdf::ModelInterfaceSharedPtr model,std::vector<urdf::LinkConstSharedPtr> &links_phys,std::vector<boost::shared_ptr<CollMesh> > &meshes);
    void setSensors();
    bool measurementModel(std::vector<CollisionPF::Particle> &part, std::vector<KDL::Wrench> forces,double alpha);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    std::vector<KDL::Wrench> jointsToMeasures(const sensor_msgs::JointState msg);
    std::vector<CollisionPF::Particle> resampleParts(std::vector<CollisionPF::Particle> part,double percentage);
    std::vector<CollisionPF::Particle> addNoise(std::vector<CollisionPF::Particle> part,std::vector<double> std_dev);
    void createParticles(std::vector<CollisionPF::Particle>::iterator start,std::vector<CollisionPF::Particle>::iterator end,std::vector<CollisionPF::Particle> range);
    std::vector<CollisionPF::Particle> motionModel(std::vector<CollisionPF::Particle> parts, std::vector<KDL::Frame> prev);
    bool partReturnCallback(robot_collision_detection::GetParts::Request& request, robot_collision_detection::GetParts::Response& response);
    bool restartEstimation(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool stepEstimation(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool pauseEstimation(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    void publishOutputs(std::vector<CollisionPF::Particle> parts);
};


#endif //ROBOT_COLLISION_DETECTION_COLLISION_PF_H
