//
// Created by Joao Bimbo on 07/12/17.
//

#include <ros/ros.h>
#include <robot_collision_detection/CollisionPF.h>




int main(int argc, char** argv) {

    ros::init(argc, argv, "robot_collision_detection");

    CollisionPF* col = new CollisionPF();
    col->run();

    return 0;
}