//
// Created by joao on 01/02/19.
//

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo/gazebo_client.hh>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>


class ContactDisplay {
public:
    ContactDisplay(){
        n_=new ros::NodeHandle();
        //node = new gazebo::transport::Node();
        node_=gazebo::transport::NodePtr(new gazebo::transport::Node());
        node_->Init();
        rrate_ = new ros::Rate(5);
        pub_point_ = n_->advertise<geometry_msgs::PointStamped>("contact_point", 1);
        pub_point_local_ = n_->advertise<geometry_msgs::PointStamped>("contact_point_local", 1);
        pub_force_ = n_->advertise<geometry_msgs::WrenchStamped>("contact_force", 1);
        pub_arrows_ = n_->advertise<visualization_msgs::MarkerArray>("contact_force_arrows", 1);
        pub_js_ = n_->advertise<sensor_msgs::JointState>("fake_joint_states", 1);
        sub_states_ = n_->subscribe("/gazebo/link_states",1, &ContactDisplay::linkStatesCb,this);
        sub_joints_ = n_->subscribe("/iiwa/joint_states",1, &ContactDisplay::fakeJsCb,this);
        sub_ = node_->Subscribe("/gazebo/default/physics/contacts", &ContactDisplay::forcesCb,this);
        ros::param::param<double>("/robot_collision_detection/sensor_noise", this->noise_std_, 1.0);


    }

    void run();
    void wait();

protected:
    gazebo::transport::SubscriberPtr sub_;
    ros::NodeHandle *n_;
    ros::Rate *rrate_;
    gazebo::transport::NodePtr node_;
    gazebo::msgs::Contacts contacts_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher pub_arrows_,pub_point_,pub_force_,pub_js_,pub_point_local_;
    ros::Subscriber sub_joints_,sub_states_;
    std::vector<KDL::Frame> link_poses;
    geometry_msgs::PointStamped p_local_;
    double noise_std_;

    void linkStatesCb(const gazebo_msgs::LinkStates::ConstPtr &msg){
        link_poses.clear();
        int offset=0;
        int i=0;
        while(msg->name.at(i)!="iiwa::iiwa_link_0"){
            i++;
        }
        while(msg->name.at(i)!="iiwa::iiwa_link_7"){
            link_poses.push_back(KDL::Frame(KDL::Rotation::Quaternion(msg->pose.at(i).orientation.x,
                                                                      msg->pose.at(i).orientation.y,
                                                                      msg->pose.at(i).orientation.z,
                                                                      msg->pose.at(i).orientation.w),
                                            KDL::Vector(msg->pose.at(i).position.x,
                                                        msg->pose.at(i).position.y,
                                                        msg->pose.at(i).position.z)));
            i++;
        }
        link_poses.push_back(KDL::Frame(KDL::Rotation::Quaternion(msg->pose.at(i).orientation.x,
                                                                  msg->pose.at(i).orientation.y,
                                                                  msg->pose.at(i).orientation.z,
                                                                  msg->pose.at(i).orientation.w),
                                        KDL::Vector(msg->pose.at(i).position.x,
                                                    msg->pose.at(i).position.y,
                                                    msg->pose.at(i).position.z)));
    }

    visualization_msgs::Marker arrow_marker(gazebo::msgs::Vector3d p, gazebo::msgs::Vector3d f, double scale,std::string ns) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id="world";
        marker.color.a=0.9;
        marker.color.r=0.9;
        marker.pose.orientation.w=1.0;
        marker.ns=ns;
        marker.lifetime=ros::Duration(1.0);

        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 2 * scale;
        marker.scale.y = 4 * scale;
        marker.points.resize(2);
        marker.points.at(1).x = p.x();
        marker.points.at(1).y = p.y();
        marker.points.at(1).z = p.z();
        marker.points.at(0).x = p.x() - f.x()* scale;
        marker.points.at(0).y = p.y() - f.y()* scale;
        marker.points.at(0).z = p.z() - f.z()* scale;

        return marker;
    }
    void fakeJsCb(const sensor_msgs::JointState::ConstPtr& _msg) {
        joint_state_=*_msg;
    }
    void forcesCb(ConstContactsPtr &_msg) {
        contacts_=*_msg;
    }


    sensor_msgs::JointState create_fake_js(sensor_msgs::JointState js, geometry_msgs::WrenchStamped fs, geometry_msgs::PointStamped ps, std::vector<KDL::Frame> frames, std::string link_name) {
        sensor_msgs::JointState out = js;
        //ROS_WARN_STREAM(link_name << "!!" << link_name.c_str()[16]);
        KDL::Wrench wrench;
        KDL::Vector vector(ps.point.x,ps.point.y,ps.point.z);
        KDL::Vector p_local;
        tf::wrenchMsgToKDL(fs.wrench,wrench);
        int link_nr=(int) link_name.c_str()[16] - '0';
        if(link_nr<0 || link_nr>7) return out;
        std::mt19937 mt_rand(time(0));
        std::normal_distribution<double> rand_eff(0,this->noise_std_);

        p_local=frames.at(link_nr).Inverse()*vector;
        tf::pointKDLToMsg(p_local,this->p_local_.point);
        this->p_local_.header.frame_id=link_name.substr(6,11);

        wrench.torque=KDL::Vector(0,0,0);
        for (int i = 0; i < out.effort.size(); i++) {
            if(i<link_nr){
                KDL::Vector p_i=frames.at(i+1).Inverse()*vector;
                KDL::Vector f_i=frames.at(i+1).Inverse()*wrench.force;
                out.effort.at(i)=-(p_i.x()*f_i.y()-p_i.y()*f_i.x())+rand_eff(mt_rand);
                //KDL::Wrench wrench1=frames.at(i+1).Inverse()*-wrench;
                //out.effort.at(i)=wrench1.torque.z()+rand_eff(mt_rand);
            }
            else{
                out.effort.at(i)=0+rand_eff(mt_rand);
            }
            //frames.at(i).Inverse()
        }

        //p.header.

        return out;
    }
};

void ContactDisplay::wait(){
    while(link_poses.size()==0 || joint_state_.name.size()==0 || contacts_.contact_size()==0) {
        ros::spinOnce();
        rrate_->sleep();
    }
    }

void ContactDisplay::run() {
    while (ros::ok()){
        ros::spinOnce();
        rrate_->sleep();
        ROS_DEBUG_STREAM("N contacts: " << contacts_.contact_size());
        visualization_msgs::MarkerArray markerArray;
        geometry_msgs::PointStamped ps;
        geometry_msgs::WrenchStamped fs;
        sensor_msgs::JointState js;
        js=this->joint_state_;
        for (unsigned int i=0;i<contacts_.contact_size();i++){
            if (std::strncmp(contacts_.contact(i).collision2().c_str(),"ground_plane",10)!=0){
                gazebo::msgs::Vector3d p=contacts_.contact(i).position().Get(0);
                gazebo::msgs::Vector3d f=contacts_.contact(i).wrench(0).body_1_wrench().force();
                markerArray.markers.push_back(this->arrow_marker(p,f,0.002,contacts_.contact(i).collision2()));
                ps.point.x=p.x();ps.point.y=p.y();ps.point.z=p.z();
                fs.wrench.force.x=f.x();fs.wrench.force.y=f.y();fs.wrench.force.z=f.z();

                js = this->create_fake_js(joint_state_, fs, ps, link_poses, contacts_.contact(i).collision2());

                fs.header.stamp=ros::Time::now();
                fs.header.frame_id="world";
                ps.header=fs.header;
                this->p_local_.header.stamp=ros::Time::now();
                this->pub_force_.publish(fs);
                this->pub_point_.publish(ps);
                this->pub_arrows_.publish(markerArray);
                this->pub_point_local_.publish(this->p_local_);
            }
        }
        this->pub_js_.publish(js);

    }


}

int main(int _argc, char **_argv) {
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "force_measure");

    ContactDisplay cd;
    cd.wait();
    cd.run();



}
