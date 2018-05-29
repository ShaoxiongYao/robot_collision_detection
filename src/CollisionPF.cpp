//
// Created by Joao Bimbo on 07/12/17.
//

#include <robot_collision_detection/CollisionPF.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>     // Post processing flags
#include <assimp/scene.h>           // Output data structure
#include <ros/package.h>
#include <string>

void CollisionPF::init(){
    ros::param::param<std::string>(ns_+"/robot_description_param", this->description_param_, "/robot_description");
    ros::param::param<std::string>(ns_+"/base_frame", this->base_frame_, "world");
    ros::param::param<std::string>(ns_+"/ee_frame", this->ee_frame_, "iiwa_link_7");
    ros::param::param<double>(ns_+"/frequency", this->freq_, 10.0);
    ros::param::param<std::string>(this->description_param_, robot_description_xml_, "");

    kdl_parser::treeFromParam(this->description_param_,robot_tree_);
    robot_tree_.getChain(this->base_frame_,ee_frame_,this->robot_chain_);
    rate_=new ros::Rate(freq_);

    urdf_model_ = urdf::parseURDF(robot_description_xml_);

    std::vector<urdf::LinkConstSharedPtr> links;

    links.push_back(urdf_model_->getRoot());
    ROS_WARN_STREAM(links.back()->name.c_str());
    while (links.back()->name.compare(this->ee_frame_)!=0 && links.back()->child_joints.size()!=0) {
        ROS_INFO_STREAM(links.back()->name.c_str());
        //urdf::Geometry *geom=links.back()->collision->geometry.get();


        if(links.back()->collision!=NULL){
            if (links.back()->collision->geometry->type==urdf::Geometry::MESH){
                urdf::Mesh* mesh= boost::dynamic_pointer_cast<urdf::Mesh>(links.back()->collision->geometry.get());
                // urdf::Mesh* mesh2 = dynamic_cast<urdf::Mesh*>(geom.get());


                std::size_t pos=mesh->filename.substr(10).find("/");




                  ROS_WARN_STREAM("N: " << pos << " " << mesh->filename.substr(10,pos).c_str());

                ROS_WARN_STREAM(ros::package::getPath(mesh->filename.substr(10,pos)));
                ROS_WARN_STREAM("###" << mesh->filename.substr(10+pos,-1));

                std::string mesh_path = ros::package::getPath(mesh->filename.substr(10,pos)) +
                        mesh->filename.substr(10+pos,-1);



             //   mesh->filename="/home/joao/ros_soma/src/iiwa_stack/iiwa_description"+mesh

                Assimp::Importer _importer;
                const aiScene* scene = _importer.ReadFile(mesh_path, aiProcess_SortByPType|aiProcess_Triangulate); //|aiProcess_GenNormals|aiProcess_GenUVCoords|aiProcess_FlipUVs);
                if( !scene ) {
                    ROS_WARN("failed to load resource %s",mesh_path.c_str());
                    return;
                }
                if( !scene->mRootNode ) {
                    ROS_WARN("resource %s has no data",mesh_path.c_str());
                    return;
                }
                if (!scene->HasMeshes()) {
                    ROS_WARN_STREAM("No meshes found in file %s" << mesh_path.c_str());
                    return;
                }

                ROS_ERROR_STREAM("NMES: " << scene->mNumMeshes);

                aiMesh* input_mesh = scene->mMeshes[0];
                ROS_ERROR_STREAM("NMES: " << input_mesh->mNumFaces);




            }
        }







        links.push_back(urdf_model_->getLink(links.back()->child_joints.back()->child_link_name));
    }

/*
    urdf::Mesh mesh;
    urdf::LinkConstSharedPtr link1=urdf_model_->getLink(robot_chain_.getSegment(1).getName());
    urdf::GeometryConstSharedPtr geom1=link1->collision->geometry;
*/



}


void CollisionPF::run() {
    while(ros::ok()){
        rate_->sleep();
    }
}
