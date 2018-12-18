//
// Created by joao on 02/10/18.
//

#include "robot_collision_detection/RobotController.h"
void RobotController::updateRobotState(sensor_msgs::JointState::ConstPtr in){
    ROS_INFO("Got it 2 %f",in->position.at(3));
}
