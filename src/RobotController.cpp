//
// Created by joao on 02/10/18.
//

#include <robot_collision_detection/RobotController.h>

RobotController::RobotController() {
    ros::param::param<std::string>("robot_description_param", this->description_param_, "/robot_description");
    ros::param::param<std::string>("base_frame", this->base_frame_, "iiwa_link_0");
    ros::param::param<std::string>("ee_frame", this->ee_frame_, "iiwa_link_ee");
    ros::param::param<std::string>("joint_states_topic", this->joint_states_topic_, "/iiwa/joint_states");
    ros::param::param<double>("filter_alpha", this->alpha_, 0.8);


    kdl_parser::treeFromParam(this->description_param_,robot_tree_);
    this->robot_tree_.getChain(this->base_frame_,ee_frame_,this->robot_chain_);
    this->dyn_model_.reset(new() KDL::ChainDynParam(this->robot_chain_,KDL::Vector(0,0,-9.8)));
    this->joints_.resize(this->robot_chain_.getNrOfJoints());


    this->joint_state_.header.stamp=ros::Time::now();
    //this->joint_state_.effort.resize(this->robot_chain_.getNrOfJoints());
    filt_vel.resize(this->robot_chain_.getNrOfJoints(),LowPassFilter(0.8));
    filt_acc.resize(this->robot_chain_.getNrOfJoints(),LowPassFilter(0.8));
    filt_torq.resize(this->robot_chain_.getNrOfJoints(),LowPassFilter(0.8));

    initialized_ = false;

}

void RobotController::updateRobotState(sensor_msgs::JointState::ConstPtr in){
    if(!initialized_) {
        this->joint_state_=*in;
        for (unsigned int i=0;i<in->name.size();i++) {
            this->joints_.q(i, 0)=in->position.at(i);
        }
        initialized_= true;
        return;
    }

    double t=in->header.stamp.toSec();
    double dt=(in->header.stamp-this->joint_state_.header.stamp).toSec();

    for (unsigned int i=0;i<in->name.size();i++) {
        double pos = in->position.at(i);
        //double vel = (pos - this->joints_.q(i, 0)) / dt;
        //if(vel==vel) this->joints_.qdot(i, 0) = filt_vel.at(i).filter(vel);

        double vel = in->velocity.at(i);

        double acc = (vel - this->joints_.qdot(i, 0)) / dt;

        this->joints_.q(i, 0) = pos;
        this->joints_.qdot(i, 0)=vel;

        if(acc==acc) this->joints_.qdotdot(i, 0) = filt_acc.at(i).filter(acc);
        //this->joints_.qdot(i, 0) = alpha_ * vel + (1 - alpha_) * this->joints_.qdot(i, 0);
        //this->joints_.qdotdot(i, 0) = alpha_ * acc + (1 - alpha_) * this->joints_.qdotdot(i, 0);
    }

    ROS_DEBUG("%.3f %.3f %.3f %.3f %.3f %.3f %.3f",this->joints_.qdot(0, 0),this->joints_.qdot(1, 0),this->joints_.qdot(2, 0),
             this->joints_.qdot(3, 0),this->joints_.qdot(4, 0),this->joints_.qdot(5, 0),this->joints_.qdot(6, 0));

    KDL::JntArray t_grav(this->robot_chain_.getNrOfJoints()),t_cor(this->robot_chain_.getNrOfJoints()),t_acc(this->robot_chain_.getNrOfJoints()),t_comp(this->robot_chain_.getNrOfJoints());

    KDL::JntSpaceInertiaMatrix inertiaMatrix(this->robot_chain_.getNrOfJoints());
    this->dyn_model_->JntToGravity(this->joints_.q,t_grav);
    //this->dyn_model_->JntToCoriolis(this->joints_.q,this->joints_.qdot,t_cor);
    this->dyn_model_->JntToMass(this->joints_.q,inertiaMatrix);
    KDL::Multiply(inertiaMatrix,this->joints_.qdotdot,t_acc);

    KDL::Add(t_acc,t_cor,t_comp);
    if (t_comp==t_comp){

    }
    else{
        t_comp.data.setZero();
    }
    KDL::Add(t_comp,t_grav,t_comp);
    sensor_msgs::JointState prev=this->joint_state_;

    this->joint_state_=*in;

    for (unsigned int i=0;i<in->name.size();i++) {
        this->joint_state_.velocity.at(i)=this->joints_.qdot(i, 0);
        double torque=this->filt_torq.at(i).filter(in->effort.at(i));
        this->joint_state_.effort.at(i)=torque-t_comp(i,0);
    }
}
