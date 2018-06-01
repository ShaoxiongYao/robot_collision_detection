//
// Created by joao on 30/05/18.
//

#include <robot_collision_detection/CollMesh.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_kdl/tf2_kdl.h>
#include <kdl_conversions/kdl_msg.h>


CollMesh::CollMesh(aiMesh* in_mesh){
    //memcpy(&(this->ass_mesh_),in_mesh,sizeof(in_mesh));
    /*this->ass_mesh_= malloc(sizeof(*(this->ass_mesh_)));
    memcpy((this->ass_mesh_->mFaces),in_mesh->mFaces,sizeof(unsigned  int)*in_mesh->mNumFaces*in_mesh->mFaces->mNumIndices);
    memcpy((this->ass_mesh_->mVertices),in_mesh->mVertices,sizeof(aiVector3D)*in_mesh->mNumVertices);
    memcpy((this->ass_mesh_->mNormals),in_mesh->mNormals,sizeof(aiVector3D)*in_mesh->mNumFaces);
*/
    init(in_mesh,KDL::Frame());
}

void CollMesh::copyMesh(aiMesh* in_mesh,aiMesh* out_mesh){

}
void CollMesh::setPose(KDL::Frame in){
    this->pose_=in;
}
void CollMesh::init(aiMesh* in_mesh,KDL::Frame T){

    this->ass_mesh_.mNumFaces=in_mesh->mNumFaces;
    this->ass_mesh_.mNumVertices=in_mesh->mNumVertices;

    this->ass_mesh_.mVertices=(aiVector3D*) malloc(in_mesh->mNumVertices*sizeof(aiVector3D));
    this->ass_mesh_.mFaces=(aiFace*) malloc(in_mesh->mNumFaces*sizeof(aiFace));
    this->ass_mesh_.mNormals=(aiVector3D*) malloc(in_mesh->mNumFaces*sizeof(aiVector3D));



    for(int i=0;i<in_mesh->mNumVertices;i++){

        KDL::Vector v1,v2;
        v1=KDL::Vector(in_mesh->mVertices[i].x,in_mesh->mVertices[i].y,in_mesh->mVertices[i].z);
        v2=T*v1;
        this->ass_mesh_.mVertices[i].x=v2.x();
        this->ass_mesh_.mVertices[i].y=v2.y();
        this->ass_mesh_.mVertices[i].z=v2.z();
    }

    for(int i=0;i<in_mesh->mNumFaces;i++){
        memcpy(&(this->ass_mesh_.mFaces[i]),&(in_mesh->mFaces[i]),sizeof(aiFace));
        KDL::Vector n1,n2;
        n1=KDL::Vector(in_mesh->mNormals[in_mesh->mFaces[i].mIndices[0]].x,in_mesh->mNormals[in_mesh->mFaces[i].mIndices[0]].y,in_mesh->mNormals[in_mesh->mFaces[i].mIndices[0]].z);
        n2=T.M*n1;
        n2.Normalize();
        this->ass_mesh_.mNormals[i].x=n2.x();
        this->ass_mesh_.mNormals[i].y=n2.y();
        this->ass_mesh_.mNormals[i].z=n2.z();
//        printf("p=[%f %f %f]\t n=[%f %f %f]\n",
//               (in_mesh->mVertices[in_mesh->mFaces[i].mIndices[0]].x +
//                in_mesh->mVertices[in_mesh->mFaces[i].mIndices[1]].x +
//                in_mesh->mVertices[in_mesh->mFaces[i].mIndices[2]].x) / 3,
//               (in_mesh->mVertices[in_mesh->mFaces[i].mIndices[0]].y +
//                in_mesh->mVertices[in_mesh->mFaces[i].mIndices[1]].y +
//                in_mesh->mVertices[in_mesh->mFaces[i].mIndices[2]].y) / 3,
//               (in_mesh->mVertices[in_mesh->mFaces[i].mIndices[0]].z +
//                in_mesh->mVertices[in_mesh->mFaces[i].mIndices[1]].z +
//                in_mesh->mVertices[in_mesh->mFaces[i].mIndices[2]].z) / 3,
//               in_mesh->mNormals[in_mesh->mFaces[i].mIndices[0]].x,
//               in_mesh->mNormals[in_mesh->mFaces[i].mIndices[0]].y,
//               in_mesh->mNormals[in_mesh->mFaces[i].mIndices[0]].z);


    }
    printf("M: %d F: %d\n",this->ass_mesh_.mNumVertices,this->ass_mesh_.mNumFaces);

}

CollMesh::PointCloud::Ptr CollMesh::createPointCloud(aiMesh* in_mesh,std::string name){
    CollMesh::PointCloud::Ptr cloud(new CollMesh::PointCloud);
    cloud->header.frame_id=name;
    cloud->reserve(in_mesh->mNumFaces);
    for(int i=0;i<in_mesh->mNumFaces;i++) {
        CollMesh::PType p;
        p.x = (in_mesh->mVertices[in_mesh->mFaces[i].mIndices[0]].x +
               in_mesh->mVertices[in_mesh->mFaces[i].mIndices[1]].x +
               in_mesh->mVertices[in_mesh->mFaces[i].mIndices[2]].x) / 3;
        p.y = (in_mesh->mVertices[in_mesh->mFaces[i].mIndices[0]].y +
               in_mesh->mVertices[in_mesh->mFaces[i].mIndices[1]].y +
               in_mesh->mVertices[in_mesh->mFaces[i].mIndices[2]].y) / 3;
        p.z = (in_mesh->mVertices[in_mesh->mFaces[i].mIndices[0]].z +
               in_mesh->mVertices[in_mesh->mFaces[i].mIndices[1]].z +
               in_mesh->mVertices[in_mesh->mFaces[i].mIndices[2]].z) / 3;
        p.normal_x=in_mesh->mNormals[i].x;
        p.normal_y=in_mesh->mNormals[i].y;
        p.normal_z=in_mesh->mNormals[i].z;
        p.normal[0]=in_mesh->mNormals[i].x;
        p.normal[1]=in_mesh->mNormals[i].y;
        p.normal[2]=in_mesh->mNormals[i].z;

        cloud->push_back(p);
    }
    return(cloud);
}

CollMesh::PointCloud::Ptr CollMesh::getLikelihoods(KDL::Wrench force,KDL::Wrench mask){
    //TODO: vectors p and n don't change. Do this only once.
    for(int i=0;i<pcloud_->size();i++){
        KDL::Vector t;
        KDL::Vector n(pcloud_->at(i).normal_x,pcloud_->at(i).normal_y,pcloud_->at(i).normal_z);
        KDL::Vector p(pcloud_->at(i).x,pcloud_->at(i).y,pcloud_->at(i).z);
        //if(KDL::dot(force.force/force.force.Norm(),n)>0.70) {
        t = (p*force.force) - force.torque; //TODO: Consider normal direction
        pcloud_->at(i).intensity = exp(-2.5 * KDL::dot(t, t)-5.0*(1+KDL::dot(force.force/force.force.Norm(),n)));


        // }
        //else{
        //    pcloud_->at(i).intensity=0;
        //}
    }

    return this->pcloud_;


}

CollMesh::CollMesh(std::string mesh_path, urdf::LinkConstSharedPtr link){
    //const aiScene* scene = this->importer_.ReadFile(mesh_path,0);
    const aiScene* scene = this->importer_.ReadFile(mesh_path,aiProcess_JoinIdenticalVertices | aiProcess_Triangulate | aiProcess_GenNormals);
    if( !scene ) {
        fprintf(stderr,"failed to load resource %s",mesh_path.c_str());
        return;
    }
    if( !scene->mRootNode ) {
        fprintf(stderr,"resource %s has no data",mesh_path.c_str());
        return;
    }
    if (!scene->HasMeshes()) {
        fprintf(stderr,"No meshes found in file %s", mesh_path.c_str());
        return;
    }
    //memcpy(&(this->ass_mesh_),scene->mMeshes[0],sizeof(*(scene->mMeshes[0]))-1);
    //this->ass_mesh_=*(scene->mMeshes[0]);
    this->link_name_=link->name;

    //urdf::Pose pose=link->parent_joint->parent_to_joint_origin_transform;
    //KDL::Frame T=KDL::Frame(KDL::Rotation::Quaternion(pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w),
    //                        KDL::Vector(pose.position.x,pose.position.y,pose.position.z));

    urdf::Pose pose2=link->collision->origin;
    KDL::Frame T2=KDL::Frame(KDL::Rotation::Quaternion(pose2.rotation.x,pose2.rotation.y,pose2.rotation.z,pose2.rotation.w),
                             KDL::Vector(pose2.position.x,pose2.position.y,pose2.position.z));


    //init(scene->mMeshes[0],T*T2);
    //pcloud_=createPointCloud(&(this->ass_mesh_),link->getParent()->name);

    init(scene->mMeshes[0],T2);
    pcloud_=createPointCloud(&(this->ass_mesh_),link->name);


}
aiMesh* CollMesh::getMesh(){
    return &ass_mesh_;
}

visualization_msgs::Marker CollMesh::getMarker(){
    if (marker_.points.size()==0) createMarker();
    return this->marker_;
}
sensor_msgs::PointCloud2 CollMesh::getPointCloud(){
    if(this->pcloud_->size()==0) this->pcloud_=createPointCloud(&(this->ass_mesh_),this->link_name_);
    pcl::toROSMsg(*(this->pcloud_),pointcloud2_);
    pointcloud2_.header.stamp=ros::Time::now();
    return(pointcloud2_);
}


visualization_msgs::Marker CollMesh::createMarker(){

    std_msgs::ColorRGBA colorRGBA;
    colorRGBA.a=0.3;
    colorRGBA.r=(float) std::rand()/RAND_MAX;
    colorRGBA.g=(float) std::rand()/RAND_MAX;
    colorRGBA.b=(float) std::rand()/RAND_MAX;

    for(int i=0;i<this->ass_mesh_.mNumFaces;i++){
        geometry_msgs::Point p,n;
        p.x=pcloud_->at(i).x;
        p.y=pcloud_->at(i).y;
        p.z=pcloud_->at(i).z;
        n.x=pcloud_->at(i).x+pcloud_->at(i).normal_x/100;
        n.y=pcloud_->at(i).y+pcloud_->at(i).normal_y/100;
        n.z=pcloud_->at(i).z+pcloud_->at(i).normal_z/100;

        marker_.points.push_back(p);
        marker_.points.push_back(n);
        marker_.colors.push_back(colorRGBA);
    }
    marker_.scale.x=0.001;
    marker_.scale.y=0.001;
    marker_.type=visualization_msgs::Marker::LINE_LIST;
    marker_.action=visualization_msgs::Marker::ADD;
    marker_.ns=this->link_name_;


    return this->marker_;
}
