#include <cslibs_mesh_map/cslibs_mesh_map_visualization.h>
#include <cslibs_mesh_map/mesh_map_tree.h>
#include <cslibs_math_ros/tf/conversion_3d.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
using namespace cslibs_math_3d;
using namespace cslibs_mesh_map;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cslibs_mesh_map_example_node");
    ros::NodeHandle n("~");

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("point_on_surface",1);
    ros::Publisher pub2 = n.advertise<visualization_msgs::MarkerArray>("random_walk",1);

    std::vector<std::string> meshes = n.param<std::vector<std::string>>("meshes",std::vector<std::string>());
    std::vector<std::string> frame_ids = n.param<std::vector<std::string>>("frame_id",std::vector<std::string>());
    std::vector<std::string> parent_ids = n.param<std::vector<std::string>>("parent_id",std::vector<std::string>());
    std::string path = n.param<std::string>("path","");
    tf::TransformListener listener;

    if((frame_ids.size() != meshes.size() && frame_ids.size() == parent_ids.size()) || frame_ids.empty()){
        throw std::runtime_error("For each mesh a frame_id is required!");
    }

    listener.waitForTransform(frame_ids.front(), frame_ids[1],
            ros::Time(0), ros::Duration(3.0));

    MeshMapTree tree;
    for(std::size_t i = 0; i < frame_ids.size(); ++i){
        std::string file = path + "/" + meshes[i];
        MeshMap link;
        if(!link.loadMeshWithNormals(file)){
            std::cerr << "Invalid input file: " << file << std::endl;
            return 42;
        }
        tf::StampedTransform transform;
        try{
            listener.lookupTransform(parent_ids[i], frame_ids[i], ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        link.frame_id_ = frame_ids[i];
        link.id_ = i;

        tree.add(parent_ids[i], link, cslibs_math_ros::tf::conversion_3d::from(transform));
    }

    MeshMapTree& l1 = tree.getNode(frame_ids.front());
    visualization_msgs::MarkerArray m1;
    visualization_msgs::MarkerArray m2;
    visualization_msgs::Marker msg;
    msg.action = visualization_msgs::Marker::MODIFY;
    msg.lifetime = ros::Duration(0.2);
    msg.id = 0;
    visualization::visualizeVertices(l1.map_, msg);
    m1.markers.push_back(msg);
    //    visualization::visualizeBoundry(l1.map_, msg);
    //    m1.markers.push_back(msg);

    ros::Rate r(20);
    ros::Duration update_time(0.1);
    ros::Time last_update = ros::Time::now();
    while(ros::ok()){

        ros::Time current = ros::Time::now();
        for(visualization_msgs::Marker& m : m1.markers){
            m.header.stamp = current;
        }
        //        bool updated = (current - last_update) > update_time;
        //        auto it = particles.begin();
        //        for(visualization_msgs::Marker& m : vparticles.markers){
        //            m.header.stamp = current;
        //            if(updated){
        //                Particle& p = *it;
        //                rand.update(p, mesh);
        //                visualizeParticle(p, mesh, m);
        //            }
        //            ++it;
        //        }
        //        if(updated){
        //            last_update = current;
        //        }
        pub.publish(m1);
        //        pub2.publish(vparticles);
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
