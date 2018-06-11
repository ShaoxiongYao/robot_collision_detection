//
// Created by Joao Bimbo on 07/12/17.
//

#include <robot_collision_detection/CollisionPF.h>

void CollisionPF::load_meshes(urdf::ModelInterfaceSharedPtr model,std::vector<urdf::LinkConstSharedPtr> &links_phys,std::vector<boost::shared_ptr<CollMesh> > &meshes){
    std::vector<urdf::LinkConstSharedPtr> links;
    links.push_back(model->getRoot());
    //links.push_back(urdf_model_->getLink(links.back()->child_joints.back()->child_link_name));

    meshes.clear();
    //Load collision meshes
    while(links.back()->name.compare(this->base_frame_)!=0) {
        links.push_back(urdf_model_->getLink(links.back()->child_joints.back()->child_link_name));
    }
    while (links.back()->getParent()->name.compare(this->ee_frame_)!=0 && links.back()->child_joints.size()!=0) {
        if(links.back()->collision!=NULL){
            std::string mesh_path;
            if (links.back()->collision->geometry->type==urdf::Geometry::MESH){
                //urdf::Mesh* mesh= boost::dynamic_pointer_cast<urdf::Mesh>(links.back()->collision->geometry.get());
                std::shared_ptr<urdf::Mesh> mesh= urdf::dynamic_pointer_cast<urdf::Mesh,urdf::Geometry>(links.back()->collision->geometry);

                if (!mesh->filename.substr(0,10).compare("package://")) {
                    std::size_t pos = mesh->filename.substr(10).find("/");
                    mesh_path = ros::package::getPath(mesh->filename.substr(10, pos)) +
                                mesh->filename.substr(10 + pos, -1);
                }
                else{
                    mesh_path=mesh->filename;
                }

                links_phys.push_back((urdf::LinkConstSharedPtr) links.back());


                boost::shared_ptr<CollMesh> a(new CollMesh(mesh_path,links.back()));
                //a=new CollMesh(mesh_path);
                meshes_.push_back(a);
                printf("N: %d\n",a->getMesh()->mNumVertices);
                //       printf("O: %d\n",meshes.back().getMesh()->mNumVertices);

            }

        }
        links.push_back(urdf_model_->getLink(links.back()->child_joints.back()->child_link_name));
    }

}

void CollisionPF::init(){
    ros::param::param<std::string>(ns_+"/robot_description_param", this->description_param_, "/robot_description");
    ros::param::param<std::string>(ns_+"/joint_states_topic", this->joint_states_topic_, "/joint_states");
    ros::param::param<std::string>(ns_+"/base_frame", this->base_frame_, "iiwa_link_0");
    ros::param::param<std::string>(ns_+"/ee_frame", this->ee_frame_, "iiwa_link_7");
    ros::param::param<double>(ns_+"/frequency", this->freq_, 10.0);
    ros::param::param<std::string>(this->description_param_, robot_description_xml_, "");


    ROS_INFO("Namespace: [%s]",ns_.c_str());
    kdl_parser::treeFromParam(this->description_param_,robot_tree_);
    robot_tree_.getChain(this->base_frame_,ee_frame_,this->robot_chain_);
    this->jnt_array_.resize(this->robot_chain_.getNrOfJoints());

    sub_jointstate_=nh_->subscribe(this->joint_states_topic_,1,&CollisionPF::joint_state_callback,this);
    pub_marray_=nh_->advertise<visualization_msgs::MarkerArray>("body_markers",1);
    pub_poses_=nh_->advertise<geometry_msgs::PoseArray>("joint_poses",1);


    rate_=new ros::Rate(freq_);

    urdf_model_ = urdf::parseURDF(robot_description_xml_);
    std::vector<urdf::LinkConstSharedPtr> links;

    this->load_meshes(urdf_model_,links,meshes_);
    this->fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->robot_chain_));

}
void CollisionPF::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg){
    this->joint_state_=*msg;
    for(int i=0;i<this->robot_chain_.getNrOfJoints();i++){
        this->jnt_array_.q.data[i]=msg->position.at(i);
    }
}


visualization_msgs::MarkerArray CollisionPF::getMarkers(){
    visualization_msgs::MarkerArray markerArray;
    for(int i=0;i<this->meshes_.size();i++){
        visualization_msgs::Marker marker=this->meshes_.at(i)->getMarker();
        marker.header.frame_id=this->base_frame_;
        marker.header.stamp=ros::Time::now();
        marker.pose.orientation.w=1.0;
        marker.color.a=1.0;
        marker.color.b=1.0;
        markerArray.markers.push_back(marker);
    }
    return markerArray;
}

void CollisionPF::run() {
    std::vector<ros::Publisher> pub_pc;
    for(int i=0;i<8;i++){
        pub_pc.push_back(nh_->advertise<sensor_msgs::PointCloud2>("pcloud"+std::to_string(i),1));
    }
    unsigned seed = ros::Time::now().sec;

    //std::default_random_engine generator (seed);
    boost::random::mt19937 generator (seed);
    boost::random::normal_distribution<double> distribution (0.0,0.5);


    std::vector<double> r(6);
    while(ros::ok()){
        ros::spinOnce();
        for (int rr=0;rr<6;rr++){
         r.at(rr)=distribution(generator);
            if (rr>2) r.at(rr)/=50;
        }

        visualization_msgs::MarkerArray markerArray=this->getMarkers();
        geometry_msgs::PoseArray poseArray;
        KDL::Wrench F_in(KDL::Vector(0.0+r.at(0),10.0+r.at(1),0+r.at(2)),KDL::Vector(-03.0+r.at(3),0.0+r.at(4),-0.00+r.at(5)));

        for (int i=0;i<this->meshes_.size() ;i++) {

            KDL::Frame T;
            geometry_msgs::Pose m;
            fk_solver_->JntToCart(this->jnt_array_.q, T,i);
            this->meshes_.at(i)->setPose(T);
            tf::poseKDLToMsg(T, m);
            poseArray.poses.push_back(m);
            markerArray.markers.at(i).pose=m;
            this->meshes_.at(i)->getLikelihoods(T.Inverse(F_in),KDL::Wrench());
            pub_pc.at(i).publish(this->meshes_.at(i)->getPointCloud());
        }

        poseArray.header.stamp=ros::Time::now();
        poseArray.header.frame_id="world";
        pub_marray_.publish(markerArray);
        pub_poses_.publish(poseArray);
        rate_->sleep();
    }
}
