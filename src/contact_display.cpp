//
// Created by joao on 01/02/19.
//

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/WrenchStamped.h>



class ContactDisplay {
public:
    ContactDisplay(){
        n_=new ros::NodeHandle();
        //node = new gazebo::transport::Node();
        node_=gazebo::transport::NodePtr(new gazebo::transport::Node());
        node_->Init();
        rrate_ = new ros::Rate(1);

        pub_arrows_ = n_->advertise<visualization_msgs::MarkerArray>("contact_force_arrows", 1);
        sub_ = node_->Subscribe("/gazebo/default/physics/contacts", &ContactDisplay::forcesCb,this);
    }

    void run();

protected:
    gazebo::transport::SubscriberPtr sub_;
    ros::NodeHandle *n_;
    ros::Rate *rrate_;
    gazebo::transport::NodePtr node_;
    gazebo::msgs::Contacts contacts_;
    ros::Publisher pub_arrows_;


    visualization_msgs::Marker arrow_marker(gazebo::msgs::Vector3d p, gazebo::msgs::Vector3d f, double scale,std::string ns) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id="world";
        marker.color.a=0.9;
        marker.color.r=0.9;
        marker.pose.orientation.w=1.0;
        marker.ns=ns;

        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 1 * scale;
        marker.scale.y = 2 * scale;
        marker.points.resize(2);
        marker.points.at(1).x = p.x();
        marker.points.at(1).y = p.y();
        marker.points.at(1).z = p.z();
        marker.points.at(0).x = p.x() - f.x()* scale;
        marker.points.at(0).y = p.y() - f.y()* scale;
        marker.points.at(0).z = p.z() - f.z()* scale;


        return marker;
    }


    void forcesCb(ConstContactsPtr &_msg) {
        contacts_=*_msg;
    }

};

void ContactDisplay::run() {
    while (ros::ok()){
        ros::spinOnce();
        rrate_->sleep();
        ROS_DEBUG_STREAM("N contacts: " << contacts_.contact_size());
        visualization_msgs::MarkerArray markerArray;
        for (unsigned int i=0;i<contacts_.contact_size();i++){
            if (std::strncmp(contacts_.contact(i).collision2().c_str(),"ground_plane",10)!=0){

                gazebo::msgs::Vector3d p=contacts_.contact(i).position().Get(0);
                gazebo::msgs::Vector3d f=contacts_.contact(i).wrench(0).body_1_wrench().force();
                markerArray.markers.push_back(this->arrow_marker(p,f,0.01,contacts_.contact(i).collision2()));

            }
        }
        this->pub_arrows_.publish(markerArray);
    }


}

int main(int _argc, char **_argv) {
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "force_measure");

    ContactDisplay cd;
    cd.run();



}