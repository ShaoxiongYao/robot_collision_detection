//
// Created by joao on 02/10/18.
//

#ifndef ROBOT_COLLISION_DETECTION_ROBOTCONTROLLER_H
#define ROBOT_COLLISION_DETECTION_ROBOTCONTROLLER_H

#include <ros/ros.h>
#include <kdl/chaindynparam.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/scoped_ptr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarrayacc.hpp>
#include <robot_collision_detection/filters.h>

class RobotController {

public:
    RobotController();
    void updateRobotState(sensor_msgs::JointState::ConstPtr in);
    sensor_msgs::JointState getJointStates(){return this->joint_state_;};
    bool initialized_;

protected:
    KDL::Chain robot_chain_;
    KDL::Tree robot_tree_;
    KDL::JntArrayAcc joints_;
    std::string base_frame_,ee_frame_,description_param_,joint_states_topic_;

    std::vector<LowPassFilter> filt_vel,filt_acc,filt_torq;

    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    boost::scoped_ptr<KDL::ChainDynParam> dyn_model_;
    sensor_msgs::JointState joint_state_;
    double alpha_;


};


#endif //ROBOT_COLLISION_DETECTION_ROBOTCONTROLLER_H
