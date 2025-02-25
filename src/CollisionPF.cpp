//
// Created by Joao Bimbo on 07/12/17.
//

#include <robot_collision_detection/CollisionPF.h>

void CollisionPF::loadMeshes(urdf::ModelInterfaceSharedPtr model,
                             std::vector<urdf::LinkConstSharedPtr> &links_phys,
                             std::vector<std::shared_ptr<CollMesh> > &meshes){
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
                // auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(links.back()->collision->geometry.get());
                std::shared_ptr<urdf::Mesh> mesh= urdf::dynamic_pointer_cast<urdf::Mesh,urdf::Geometry>(links.back()->collision->geometry);
                if (mesh->filename.substr(0,10)=="package://") {
                    std::size_t pos = mesh->filename.substr(10).find('/');
                    mesh_path = ros::package::getPath(mesh->filename.substr(10, pos)) +
                                mesh->filename.substr(10 + pos, -1);
                }
                else{
                    mesh_path=mesh->filename;
                }

                links_phys.push_back((urdf::LinkConstSharedPtr) links.back());


                std::shared_ptr<CollMesh> a(new CollMesh(mesh_path,links.back()));
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

bool CollisionPF::pauseEstimation(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    this->loadParameters();
    this->run_=!this->run_;
}


bool CollisionPF::stepEstimation(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    this->run_= false;
    this->loadParameters();
    std::vector<KDL::Wrench> measurements = this->jointsToMeasures(this->joint_state_);
    this->step(this->parts, measurements,1,1);
    this->publishOutputs(this->parts);
    return true;
}

bool CollisionPF::restartEstimation(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

    this->parts.resize(this->num_parts_);
    this->loadParameters();
    ROS_WARN("n: %d ranges: %d %d %f %f",this->num_parts_,this->part_ranges.at(0).n, this->part_ranges.at(1).n,this->part_ranges.at(0).F,this->part_ranges.at(1).F);
    this->createParticles(parts.begin(), parts.end(), this->part_ranges);
    this->run_=true;
    return true;
}

bool CollisionPF::partReturnCallback(robot_collision_detection::GetParts::Request& request, robot_collision_detection::GetParts::Response& response){
    for (unsigned int i=0;i<this->parts.size();i++){
        robot_collision_detection::CollPart part;
        part.n=this->parts.at(i).n;
        part.p_idx=this->parts.at(i).p;
        tf::wrenchKDLToMsg(this->meshes_.at(this->parts.at(i).n)->ForceAtPoint(this->parts.at(i).p,this->parts.at(i).F,0.0),part.F);
        part.p.x=this->meshes_.at(this->parts.at(i).n)->pcloud_->at(this->parts.at(i).p).x;
        part.p.y=this->meshes_.at(this->parts.at(i).n)->pcloud_->at(this->parts.at(i).p).y;
        part.p.z=this->meshes_.at(this->parts.at(i).n)->pcloud_->at(this->parts.at(i).p).z;
        part.K=parts.at(i).K;
        part.w=parts.at(i).w;
        response.part.push_back(part);
    }
    for (unsigned int i=0;i<this->meshes_.size();i++){
        geometry_msgs::Pose j_pose;
        tf::poseKDLToMsg(this->meshes_.at(i)->getPose(),j_pose);
        response.joint_poses.push_back(j_pose);
    }
    response.joint_state=this->joint_state_;
    response.header.stamp=ros::Time::now();
    if(response.part.size()>0) return true;
    else return false;
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
std::vector<CollisionPF::Particle> CollisionPF::motionModel(std::vector<CollisionPF::Particle> parts, std::vector<KDL::Frame> prev){
    std::vector<CollisionPF::Particle> p_out;
    std::vector<KDL::Frame> dFrame;
    p_out.reserve(parts.size());
    dFrame.reserve(prev.size());

    for (unsigned long i=0;i<prev.size();i++){
        dFrame.push_back(this->meshes_.at(i)->getPose().Inverse()*prev.at(i));
    }

    srand (time(NULL));
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    for (unsigned long i=0;i<parts.size();i++){

        CollMesh::PType p_prev=this->meshes_.at(parts.at(i).n)->pcloud_->at(parts.at(i).p);
        KDL::Vector p(p_prev.x,p_prev.y,p_prev.z);
        KDL::Vector n(p_prev.normal_x,p_prev.normal_y,p_prev.normal_z);
        p=dFrame.at(parts.at(i).n)*p;
        n=dFrame.at(parts.at(i).n).M*n;

        double r=pow(10,-3+3*((double) rand())/RAND_MAX);
        KDL::Vector c_v=p+(r*n);
        CollMesh::PType c;
        c.x=c_v.x();c.y=c_v.y();c.z=c_v.z();

        this->meshes_.at(parts.at(i).n)->kdtree_.nearestKSearch(c,1,pointIdxNKNSearch,pointNKNSquaredDistance);
        CollisionPF::Particle particle=parts.at(i);
        particle.F+=particle.K*(r-sqrt(pointNKNSquaredDistance.at(0)));
        particle.p=pointIdxNKNSearch.at(0);
        p_out.push_back(particle);
    }
    return(p_out);
}

std::vector<CollisionPF::Particle> CollisionPF::addNoise(std::vector<CollisionPF::Particle> part,std::vector<double> std_dev){
    std::vector<CollisionPF::Particle> out(part.size());
    std::mt19937 mt_rand(time(0));

    std::normal_distribution<double> rand_p(0,std_dev.at(1));
    std::normal_distribution<double> rand_F(0,std_dev.at(2));
    std::normal_distribution<double> rand_K(0,std_dev.at(3));
    srand (time(NULL));



    for (unsigned long i=0;i<part.size();i++){
        int r =rand();
        if(r<RAND_MAX*std_dev.at(0) && part.at(i).n>0) out.at(i).n=part.at(i).n-1;
        else if (r>(1-std_dev.at(0))*RAND_MAX && part.at(i).n<this->meshes_.size()-2) out.at(i).n=part.at(i).n+1;
        else out.at(i).n=part.at(i).n;

        out.at(i).F=fabs(part.at(i).F+rand_F(mt_rand));

        //Different way of finding a near point
        CollMesh::PType p(this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p));
        p.x+=rand_p(mt_rand);
        p.y+=rand_p(mt_rand);
        p.z+=rand_p(mt_rand);
        out.at(i).p=this->meshes_.at(out.at(i).n)->getNearestKidx(1,p).at(0);

        //std::vector<int> idx=this->meshes_.at(part.at(i).n)->getPointsInRadius(this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p),std_dev.at(1));
        //out.at(i).p=idx.at(mt_rand()%(idx.size())); //Random between 0 and idx.size()

        out.at(i).K=fabs(part.at(i).K+rand_K(mt_rand));
        out.at(i).w=part.at(i).w;
    }
    return (out);

}
void CollisionPF::createParticles(std::vector<CollisionPF::Particle>::iterator start,std::vector<CollisionPF::Particle>::iterator end,std::vector<CollisionPF::Particle> range){
    for (std::vector<CollisionPF::Particle>::iterator it=start;it!=end;it++){
        it->n = (int) std::rand()%(range.at(1).n-range.at(0).n)+range.at(0).n;
        it->p = (int) std::rand()/(RAND_MAX/(this->meshes_.at(it->n)->getMeshSize()-1));
        it->F = ((double) std::rand()/((double) RAND_MAX))*(range.at(1).F-range.at(0).F)+range.at(0).F;
        it->K = ((double) std::rand()/((double) RAND_MAX))*(range.at(1).K-range.at(0).K)+range.at(0).K;
        it->w = ((double) std::rand()/((double) RAND_MAX))*(range.at(1).w-range.at(0).w)+range.at(0).w;
    }
}
std::vector<CollisionPF::Particle> CollisionPF::resampleParts(std::vector<CollisionPF::Particle> part,double percentage){
    //std::vector<CollisionPF::Particle> out(part.begin(),part.begin()+(part.size()*std::ceil(part.size()*percentage)));
    std::vector<CollisionPF::Particle> out(std::ceil(part.size()*percentage));
    std::mt19937 mt_rand(time(0));

    std::vector<unsigned long> r(out.size());
    std::generate(r.begin(), r.end(), mt_rand);
    std::sort(r.begin(),r.end());
    unsigned long max=std::mt19937::max();
    unsigned long sum=part.begin()->w*max;
    unsigned long i=1,k=0;
    while (k<out.size() && i<part.size()){
        if(sum>r.at(k)){
            out.at(k)=part.at(i-1);
            k++;
        }
        else{
            sum+=part.at(i).w*max;
            i++;
        }
    }
//    std::vector<int> hist(part.size()+1);
//    for (int i=0;i<out.size();i++){
//        hist.at(out.at(i).n)++;
//    }
//    for (int i=0;i<hist.size();i++){
//        ROS_WARN("%d: %d",i,hist.at(i));
//    }

    return out;
}


bool CollisionPF::measurementModel(std::vector<CollisionPF::Particle> &part, std::vector<KDL::Wrench> forces,double alpha){
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
        //KDL::Wrench f=this->meshes_.at(part.at(i).n)->getPose()*KDL::Wrench(KDL::Vector(-this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_x,
        //                                                                                -this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_y,
        //                                                                                -this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_z)*part.at(i).F,
        //                                                                    KDL::Vector(0,0,0));



        KDL::Wrench f=this->meshes_.at(part.at(i).n)->getPose()*this->meshes_.at(part.at(i).n)->ForceAtPoint(part.at(i).p,part.at(i).F,0.0);


        for(unsigned long j=0;j<forces.size();j++){
            KDL::Wrench f_part;
            if (part.at(i).n>=j){

                f_part=this->meshes_.at(j)->ForceToMeasurement(f);

                /*      Eigen::Vector3d n_r(this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_x,
                                          this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_y,
                                          this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p).normal_z);
                      //CollMesh::PType p(pcl::transformPoint(this->meshes_.at(part.at(i).n)->pcloud_->points.at(part.at(i).p),T.at(part.at(i).n)));
                      CollMesh::PType p(pcl::transformPoint(this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p),T.at(part.at(i).n)));
                      n_r=T.at(part.at(i).n).rotation()*n_r;
                      p.normal_x=n_r[0];p.normal_y=n_r[1];p.normal_z=n_r[2]; //TODO: pcl::transformPointWithNormal(p,T.at(part.at(i).n));

                      CollMesh::PType p_local(pcl::transformPoint(this->meshes_.at(part.at(i).n)->pcloud_->at(part.at(i).p),T.at(j).inverse()));
                      KDL::Wrench f_local=this->meshes_.at(part.at(i).n)->getPose().Inverse(f);

                      KDL::Wrench f_part=this->meshes_.at(j)->ForceToMeasurement(p_local,f_local.force,0);
      */
            }
            else{
                f_part=KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0));
                //A force here creates no torque in joint j
            }

            KDL::Wrench diff=f_part-forces.at(j);
            double cost=diff.force.Norm()+diff.torque.Norm();
            bel*=exp(-alpha*cost);
            //if (exp(-cost)>0.9) ROS_INFO("%d %f",i, exp(-cost));
        }
        _bel.at(i)=bel*part.at(i).w;
        eta+=_bel.at(i);
    }


    double debug_sum=0;

    for(unsigned long i=0;i<part.size();i++){
        part.at(i).w=_bel.at(i)/eta; //normalise
        debug_sum+=part.at(i).w;
    }
    //ROS_WARN("sum: %f = 1.0?",debug_sum );
}

bool CollisionPF::loadParameters(){
    ros::param::param<std::string>(ns_+"/robot_description_param", this->description_param_, "/robot_description");
    ros::param::param<std::string>(ns_+"/joint_states_topic", this->joint_states_topic_, "/joint_states");
    ros::param::param<std::string>(ns_+"/base_frame", this->base_frame_, "iiwa_link_0");
    ros::param::param<std::string>(ns_+"/ee_frame", this->ee_frame_, "iiwa_link_7");
    ros::param::param<double>(ns_+"/frequency", this->freq_, 10.0);
    ros::param::param<std::vector<int> >(ns_+"/sensors", this->sensor_types, std::vector<int>(7));
    ros::param::param<std::string>(this->description_param_, robot_description_xml_, "");
    ros::param::param<int>(ns_+"/pf/num_parts",this->num_parts_, 10000);
    ros::param::param<double>(ns_+"/pf/search_alpha",this->e_alpha_, 0.001);
    ros::param::param<double>(ns_+"/pf/perc_new",this->perc_new_, 0.99);
    ros::param::param<std::vector<double> >(ns_+"/pf/std_devs",this->std_devs_, {0.00,0.01,1.0,50.0});

    this->part_ranges.resize(2);
    std::vector<double> rang(2);
    ros::param::param<std::vector<double> >(ns_+"/pf/ranges/n",rang, {0.0,7.0});
    this->part_ranges.at(0).n=(int) rang.at(0);
    this->part_ranges.at(1).n=(int) rang.at(1);

    ros::param::param<std::vector<double> >(ns_+"/pf/ranges/F",rang, {0.0,200.0});
    this->part_ranges.at(0).F=rang.at(0);
    this->part_ranges.at(1).F=rang.at(1);

    ros::param::param<std::vector<double> >(ns_+"/pf/ranges/K",rang, {0.0,10000.0});
    this->part_ranges.at(0).K=rang.at(0);
    this->part_ranges.at(1).K=rang.at(1);
    this->part_ranges.at(0).w=1/(double) this->num_parts_;
    this->part_ranges.at(1).w=1/(double) this->num_parts_;

    return true;
}


void CollisionPF::init(){
    this->loadParameters();
    ROS_INFO("Namespace: [%s]",ns_.c_str());
    if (!kdl_parser::treeFromParam(this->description_param_,robot_tree_)) exit(-1);
    if (!robot_tree_.getChain(this->base_frame_,ee_frame_,this->robot_chain_)) exit(-2);
    this->jnt_array_.resize(this->robot_chain_.getNrOfJoints());
    rate_=new ros::Rate(freq_);

    urdf_model_ = urdf::parseURDF(robot_description_xml_);
    std::vector<urdf::LinkConstSharedPtr> links;

    if (urdf_model_== nullptr) {ROS_ERROR("URDF model not loaded!"); throw;}

    this->loadMeshes(urdf_model_,links,meshes_);
    this->fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->robot_chain_));
    this->setSensors();


    sub_jointstate_=nh_->subscribe(this->joint_states_topic_,1,&CollisionPF::jointStateCallback,this);
    pub_marray_=nh_->advertise<visualization_msgs::MarkerArray>("body_markers",1);
    pub_poses_=nh_->advertise<geometry_msgs::PoseArray>("joint_poses",1);
    pub_pc_=nh_->advertise<sensor_msgs::PointCloud2>("coll_likelihood",1);
    srv_parts_ = nh_->advertiseService("get_particles",&CollisionPF::partReturnCallback, this);
    srv_restart_ = nh_->advertiseService("restart_estimation",&CollisionPF::restartEstimation, this);
    srv_step_ = nh_->advertiseService("step_estimation",&CollisionPF::stepEstimation, this);
    srv_pause_ = nh_->advertiseService("pause_estimation",&CollisionPF::pauseEstimation, this);

    this->prev_state_.clear();
    for (unsigned long i = 0; i < this->meshes_.size(); i++) {
        this->prev_state_.push_back(this->meshes_.at(i)->getPose()); //TODO: Poses aren't available yet!
    }

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

std::vector<KDL::Wrench> CollisionPF::jointsToMeasures(const sensor_msgs::JointState msg){
    std::vector<KDL::Wrench> measurements(sensor_types.size());
    // if(msg.effort.size()!=this->sensor_types.size()){
    //throw std::range_error("Measurement size is wrong") ;
    //    return measurements;
    // }
    if(msg.effort.size()==0)
        return measurements;


    measurements.at(0)=KDL::Wrench();


    for (unsigned int i=1;i<measurements.size();i++){
        KDL::Wrench m;
        switch (sensor_types.at(i)){
            case 4:
                m.torque.x(-msg.effort.at(i-1));
                break;
            case 5:
                m.torque.y(-msg.effort.at(i-1));
                break;
            case 6:
                m.torque.z(-msg.effort.at(i-1));
                break;
        }
        measurements.at(i)=m;
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


void CollisionPF::step(std::vector<CollisionPF::Particle> &parts, std::vector<KDL::Wrench> measurements,double mult_stdev, double mult_alpha) {
    ros::spinOnce();
    this->createParticles(parts.begin()+(int) (this->perc_new_*parts.size()),parts.end(),part_ranges);

    this->parts=this->motionModel(this->parts,this->prev_state_);
    this->measurementModel(parts,measurements,this->e_alpha_);
    parts=this->resampleParts(parts,1.0);
    parts=this->addNoise(parts,this->std_devs_);
    for (unsigned long i = 1; i < std_devs_.size(); i++) {
        std_devs_.at(i) *= mult_stdev;
        this->e_alpha_ *= mult_alpha;
    }
    this->prev_state_.clear();
    for (unsigned long i = 0; i < this->meshes_.size(); i++) {
        this->prev_state_.push_back(this->meshes_.at(i)->getPose());
    }

}

void CollisionPF::publishOutputs(std::vector<CollisionPF::Particle> parts){
    CollMesh::PointCloud::Ptr pc(this->particlesToPointCloud(parts));
    sensor_msgs::PointCloud2 pointCloud2;
    pcl::toROSMsg(*pc,pointCloud2);
    pub_pc_.publish(pointCloud2);
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
}

void CollisionPF::run() {
    unsigned seed = ros::Time::now().sec;
    std::default_random_engine generator (seed);
    std::srand(std::time(nullptr)); // use current time as seed for random generator
    parts.resize(this->num_parts_);
    this->createParticles(parts.begin(),parts.end(),this->part_ranges);
    int iters=0;
    while(ros::ok() ){
        ros::spinOnce();
        if(this->run_) {
            std::vector<KDL::Wrench> measurements = this->jointsToMeasures(this->joint_state_);
            this->step(this->parts, measurements,1,1);
            this->publishOutputs(this->parts);
            iters++;
            if (iters % 20 == 0) {
                for (unsigned long i = 1; i < std_devs_.size(); i++) {
                    std_devs_.at(i) *= 0.95;
                    this->e_alpha_ *= 1.001;
                }
            }
        }
        rate_->sleep();
    }
}
