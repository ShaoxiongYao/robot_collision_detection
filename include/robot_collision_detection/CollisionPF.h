//
// Created by Joao Bimbo on 07/12/17.
//

#ifndef ROBOT_COLLISION_DETECTION_COLLISION_PF_H
#define ROBOT_COLLISION_DETECTION_COLLISION_PF_H

#include <Eigen/Core>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>
#include <ros/ros.h>

class CollisionPF {
public:
    CollisionPF(){
        nh_=new ros::NodeHandle("~");
        ns_="robot_collision_detector";
        this->init();
    }
    CollisionPF(ros::NodeHandle *nh,std::string ns){
        nh_=nh;
        ns_=ns;
        this->init();
    }

    void run();


protected:
    ros::NodeHandle* nh_;
    std::string ns_,description_param_,base_frame_,ee_frame_,robot_description_xml_;
    KDL::Chain robot_chain_;
    KDL::Tree robot_tree_;
    urdf::ModelInterfaceSharedPtr urdf_model_;
    ros::Rate* rate_;

    double freq_;

    void init();

};


#endif //ROBOT_COLLISION_DETECTION_COLLISION_PF_H
