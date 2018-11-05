//
// Created by joao on 02/10/18.
//

#ifndef ROBOT_COLLISION_DETECTION_ROBOTCONTROLLER_H
#define ROBOT_COLLISION_DETECTION_ROBOTCONTROLLER_H

#include <ros/ros.h>
#include <kdl/chaindynparam.hpp>
#include <sensor_msgs/JointState.h>

class RobotController {

public:
    void updateRobotState(sensor_msgs::JointState::ConstPtr in);

protected:



};


#endif //ROBOT_COLLISION_DETECTION_ROBOTCONTROLLER_H
