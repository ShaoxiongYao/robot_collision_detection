//
// Created by joao on 01/02/19.
//

#include <gazebo-7/gazebo/transport/transport.hh>
#include <gazebo-7/gazebo/msgs/msgs.hh>
#include <gazebo-7/gazebo/gazebo_client.hh>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/WrenchStamped.h>



class ContactDisplay {
public:
    ContactDisplay(){
        n_=new ros::NodeHandle();
        node = new gazebo::transport::Node();
        node->Init();

        pub_arrows_ = n_->advertise<visualization_msgs::MarkerArray>("contact_force_arrows", 1);


    }
    void run(){
        rrate_->sleep();

    }

protected:

    ros::NodeHandle *n_;
    ros::Rate *rrate_;
    gazebo::transport::NodePtr node;
    ros::Publisher pub_arrows_;


    visualization_msgs::Marker arrow_marker(geometry_msgs::Point p, geometry_msgs::Wrench f, double scale) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.1 * scale;
        marker.scale.y = 0.2 * scale;
        marker.points.resize(2);
        marker.points.at(1) = p;
        marker.points.at(0).x = p - f.force.x;
        marker.points.at(0).y = p - f.force.y;
        marker.points.at(0).z = p - f.force.z;
        return marker;
    }


    void forcesCb(ConstContactsPtr &_msg) {


    }




};

int main(int _argc, char **_argv) {
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "force_measure");

    ContactDisplay contactDisplay();
    contactDisplay.run();

}