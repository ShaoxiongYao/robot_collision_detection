//
// Created by joao bimbo on 02/10/18.
//
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_collision_detection/RobotController.h>


void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg, RobotController* robot){
robot->updateRobotState(msg);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle nh;
    RobotController robot;
    ros::Subscriber sub_jointstate=nh.subscribe<sensor_msgs::JointState>("joint_states",1,boost::bind(&joint_state_callback, _1, &robot));
    //ros::Subscriber sub_jointstate=nh.subscribe<sensor_msgs::JointState>("joint_states",1,joint_state_callback);
    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}