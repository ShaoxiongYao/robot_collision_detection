//
// Created by Joao Bimbo on 07/12/17.
//

#include <robot_collision_detection/CollisionPF.h>

void CollisionPF::loadMeshes(urdf::ModelInterfaceSharedPtr model,std::vector<urdf::LinkConstSharedPtr> &links_phys,std::vector<boost::shared_ptr<CollMesh> > &meshes){
    std::vector<urdf::LinkConstSharedPtr> links;
    links.push_back(model->getRoot());
    //links.push_back(urdf_model_->getLink(links.back()->child_joints.back()->child_link_name));

    meshes.clear();
    //Load collision meshes
    while(links.back()->name!=this->base_frame_) {
        links.push_back(urdf_model_->getLink(links.back()->child_joints.back()->child_link_name));
    }
    while (links.back()->getParent()->name!=this->ee_frame_ && !links.back()->child_joints.empty()) {
        if(links.back()->collision!= nullptr){
            std::string mesh_path;
            if (links.back()->collision->geometry->type==urdf::Geometry::MESH){
                urdf::Mesh* mesh= boost::dynamic_pointer_cast<urdf::Mesh>(links.back()->collision->geometry.get());
                if (mesh->filename.substr(0,10)=="package://") {
                    std::size_t pos = mesh->filename.substr(10).find('/');
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
                //printf("N: %d\n",a->getMesh()->mNumVertices);
                //       printf("O: %d\n",meshes.back().getMesh()->mNumVertices);

            }

        }
        links.push_back(urdf_model_->getLink(links.back()->child_joints.back()->child_link_name));
    }
}

void CollisionPF::setSensors(){
    for(unsigned long i=0;i<this->sensor_types.size();i++) {
        KDL::Wrench mask(KDL::Vector(0,0,0),KDL::Vector(0,0,0));
        switch(this->sensor_types.at(i)){
            case 0:
                mask=KDL::Wrench(KDL::Vector(1,1,1),KDL::Vector(1,1,1));
                break;
            case 1:
                mask.force.x(1.0);
                break;
            case 2:
                mask.force.y(1.0);
                break;
            case 3:
                mask.force.z(1.0);
                break;
            case 4:
                mask.torque.x(1.0);
                break;
            case 5:
                mask.torque.y(1.0);
                break;
            case 6:
                mask.torque.z(1.0);
                break;
            case -1:
                break;
            default:
                break;
        }
        this->meshes_.at(i)->mask_=mask;
    }
}

CollMesh::PointCloud::Ptr CollisionPF::particlesToPointCloud(std::vector<CollisionPF::Particle> part){
    CollMesh::PointCloud::Ptr cloud(new CollMesh::PointCloud);
    cloud->header.frame_id= this->base_frame_;
    cloud->reserve(part.size());
    std::vector<Eigen::Affine3d> T;
    T.resize(this->meshes_.size()); // TODO: define number of sensors elsewhere
    for (unsigned long i=0;i<T.size();i++) {
        tf::transformKDLToEigen(this->meshes_.at(i)->getPose(), T.at(i));
    }

    for (unsigned long i=0;i<part.size();i++){

        Eigen::Vector3d n_r(-this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_x,
                            -this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_y,
                            -this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_z);

        CollMesh::PType p(pcl::transformPoint(this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p),T.at(part.at(i).n)));

        n_r=T.at(part.at(i).n).rotation()*n_r;
        p.normal_x=n_r[0];p.normal_y=n_r[1];p.normal_z=n_r[2]; //TODO: pcl::transformPointWithNormal(p,T.at(part.at(i).n));

        p.intensity=part.at(i).w;
        cloud->push_back(p);
    }
    return(cloud);
}

bool CollisionPF::measurementModel(std::vector<CollisionPF::Particle> &part, std::vector<KDL::Wrench> forces){
    if(this->meshes_.size() != forces.size()) {
        ROS_ERROR("Invalid size of measurements");
        return false;
    }

    float eta=0;
    std::vector<float> _bel(part.size());
    std::vector<Eigen::Affine3d> T;
    T.resize(this->meshes_.size()); // TODO: define number of sensors elsewhere

    for (unsigned long j=0;j<T.size();j++) {
        tf::transformKDLToEigen(this->meshes_.at(j)->getPose(), T.at(j));

    }
    for(unsigned long i=0;i<part.size();i++){
        double bel=1.0;
        KDL::Wrench f=this->meshes_.at(part.at(i).n)->getPose()*KDL::Wrench(KDL::Vector(-this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_x,
                                  -this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_y,
                                  -this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_z)*part.at(i).F,KDL::Vector(0,0,0));


        for(unsigned long j=0;j<forces.size();j++){

            if (part.at(i).n>=j){
                Eigen::Vector3d n_r(this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_x,
                                    this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_y,
                                    this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_z);
                //CollMesh::PType p(pcl::transformPoint(this->meshes_.at(part.at(i).n)->pcloud_->points.at(part.at(i).p),T.at(part.at(i).n)));
                CollMesh::PType p(pcl::transformPoint(this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p),T.at(part.at(i).n)));
                n_r=T.at(part.at(i).n).rotation()*n_r;
                p.normal_x=n_r[0];p.normal_y=n_r[1];p.normal_z=n_r[2]; //TODO: pcl::transformPointWithNormal(p,T.at(part.at(i).n));


                CollMesh::PType p_local(pcl::transformPoint(this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p),T.at(j).inverse()));
                KDL::Wrench f_local=this->meshes_.at(part.at(i).n)->getPose().Inverse(f);

                KDL::Wrench f_part=this->meshes_.at(j)->ForceToMeasurement(p_local,f_local.force,0);




                KDL::Wrench diff=f_part-forces.at(j);
                double cost=diff.force.Norm()+diff.torque.Norm();
                bel*=exp(-cost);
                ROS_INFO("%f",exp(-cost));

            }
            else{
                //A force here creates no torque in joint j
            }
        }
        _bel.at(i)=bel*part.at(i).w;
        eta+=_bel.at(i);
    }


    for(unsigned long i=0;i<part.size();i++){
        part.at(i).w=_bel.at(i)/eta; //normalise
    }


}


void CollisionPF::init(){
    ros::param::param<std::string>(ns_+"/robot_description_param", this->description_param_, "/robot_description");
    ros::param::param<std::string>(ns_+"/joint_states_topic", this->joint_states_topic_, "/joint_states");
    ros::param::param<std::string>(ns_+"/base_frame", this->base_frame_, "iiwa_link_0");
    ros::param::param<std::string>(ns_+"/ee_frame", this->ee_frame_, "iiwa_link_7");
    ros::param::param<double>(ns_+"/frequency", this->freq_, 10.0);
    ros::param::param<std::vector<int> >(ns_+"/sensors", this->sensor_types, std::vector<int>(7));
    ros::param::param<std::string>(this->description_param_, robot_description_xml_, "");


    ROS_INFO("Namespace: [%s]",ns_.c_str());
    kdl_parser::treeFromParam(this->description_param_,robot_tree_);
    robot_tree_.getChain(this->base_frame_,ee_frame_,this->robot_chain_);
    this->jnt_array_.resize(this->robot_chain_.getNrOfJoints());

    sub_jointstate_=nh_->subscribe(this->joint_states_topic_,1,&CollisionPF::jointStateCallback,this);
    pub_marray_=nh_->advertise<visualization_msgs::MarkerArray>("body_markers",1);
    pub_poses_=nh_->advertise<geometry_msgs::PoseArray>("joint_poses",1);


    rate_=new ros::Rate(freq_);

    urdf_model_ = urdf::parseURDF(robot_description_xml_);
    std::vector<urdf::LinkConstSharedPtr> links;

    this->loadMeshes(urdf_model_,links,meshes_);
    this->fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->robot_chain_));
    this->setSensors();

}
void CollisionPF::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){
    this->joint_state_=*msg;
    for(int i=0;i<this->robot_chain_.getNrOfJoints();i++){
        this->jnt_array_.q.data[i]=msg->position.at(i);
        KDL::Frame T;
        fk_solver_->JntToCart(this->jnt_array_.q, T,i+1);
        this->meshes_.at(i+1)->setPose(T);
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
    std::default_random_engine generator (seed);
    std::normal_distribution<double> distribution (0.0,0.1);
    std::vector<double> r(6);

    std::srand(std::time(nullptr)); // use current time as seed for random generator
    std::vector<CollisionPF::Particle> parts;
    parts.resize(5000);
    for(unsigned long i=0;i<parts.size();i++){
        parts.at(i).n=(int) std::rand()/(RAND_MAX/3);//std::rand()/(RAND_MAX/this->meshes_.size());
        parts.at(i).p=(int) std::rand()/(RAND_MAX/(this->meshes_.at(parts.at(i).n)->getMeshSize()-1));
        parts.at(i).F=10.0;
        parts.at(i).K=100.0;
        parts.at(i).w=0.01;
    }




    while(ros::ok()){
        ros::spinOnce();
      /*  for (unsigned long rr=0;rr<6;rr++){
         r.at(rr)=distribution(generator);
            if (rr>2) r.at(rr)/=50;
        }

        KDL::Wrench F_in(KDL::Vector(0.0+r.at(0),10.0+r.at(1),0+r.at(2)),KDL::Vector(-03.0+r.at(3),0.0+r.at(4),-0.00+r.at(5)));



        for (unsigned long i=0;i<this->meshes_.size() ;i++) {


            std::vector<KDL::Wrench> forces(this->meshes_.at(i)->pcloud_->size(),T.Inverse(F_in));
            std::vector<float> likelihoods;
            likelihoods.resize(this->meshes_.at(i)->pcloud_->size());
            this->meshes_.at(i)->getLikelihoods(forces,this->meshes_.at(i)->pcloud_,forces.at(0),likelihoods);
            this->meshes_.at(i)->setPointCloudIntensity(likelihoods);

            //pub_pc.at(i).publish(this->meshes_.at(i)->getPointCloud());




            CollMesh::PType p;
            p.x=0.01;
            p.y=0.01;
            p.z=0.1;
            p.normal_x=1.0;

            this->meshes_.at(i)->getNearestK(1,p);

        }
*/

        // Testing parts:
        std::vector<KDL::Wrench> measurements;
        measurements.push_back(KDL::Wrench(KDL::Vector(-10.0,0,0),KDL::Vector(0,-0.5,0.0)));
        measurements.push_back(KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0.0)));
        measurements.push_back(KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0.0)));
        measurements.push_back(KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0.0)));
        measurements.push_back(KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0.0)));
        measurements.push_back(KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0.0)));
        measurements.push_back(KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0.0)));
        measurements.push_back(KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0.0)));


        this->measurementModel(parts,measurements);
        CollMesh::PointCloud::Ptr pc(this->particlesToPointCloud(parts));

        sensor_msgs::PointCloud2 pointCloud2;

        pcl::toROSMsg(*pc,pointCloud2);

        pub_pc.at(0).publish(pointCloud2);

        visualization_msgs::MarkerArray markerArray=this->getMarkers();
        geometry_msgs::PoseArray poseArray;


        for (unsigned long i=0;i<this->meshes_.size() ;i++) {
            geometry_msgs::Pose m;
            tf::poseKDLToMsg(this->meshes_.at(i)->getPose(), m);
            poseArray.poses.push_back(m);
            markerArray.markers.at(i).pose=m;
        }


        poseArray.header.stamp=ros::Time::now();
        poseArray.header.frame_id=this->base_frame_;
        pub_marray_.publish(markerArray);
        pub_poses_.publish(poseArray);

        rate_->sleep();
    }
}
