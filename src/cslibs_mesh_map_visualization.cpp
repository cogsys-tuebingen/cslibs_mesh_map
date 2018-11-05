#include <cslibs_mesh_map/cslibs_mesh_map_visualization.h>
#include <cslibs_math_ros/geometry_msgs/conversion_3d.hpp>
using namespace cslibs_mesh_map;

void visualization::visualizeVertices(const MeshMap& mesh, visualization_msgs::Marker& msg)
{
    msg.header.frame_id = mesh.frame_id_;
    msg.ns = mesh.frame_id_ + "_vertices";
    msg.points.clear();
    msg.type = visualization_msgs::Marker::POINTS;
    for(auto it = mesh.begin(); it != mesh.end(); ++it){
        cslibs_math_3d::Vector3d piv = mesh.getPoint(it);
        geometry_msgs::Point point = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(piv);
        msg.points.push_back(point);

    }

    msg.color.a = 0.8;
    msg.color.r = 0.5;
    msg.color.g = 0;
    msg.color.b = 0.5;

    msg.scale.x = 0.005;
    msg.scale.y = 0.005;
    msg.scale.z = 0.005;
    ++msg.id;
}
void visualization::visualizeNormal(const cslibs_math_3d::Vector3d& point,
                                    const cslibs_math_3d::Vector3d& normal,
                                    visualization_msgs::Marker& msg)
{
    msg.points.clear();
    msg.type = visualization_msgs::Marker::ARROW;
    msg.color.a = 0.8;
    msg.color.r = 1.0;
    msg.color.g = 0;
    msg.color.b = 0;
    msg.scale.x = 0.005;
    msg.scale.y = 0.01;
    msg.scale.z = 0.01;
    msg.ns = "normal";
    msg.points.resize(2);
    cslibs_math_3d::Vector3d p2 = point + normal * 0.1;
    msg.points[0] = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(point);
    msg.points[1] = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(p2);
    ++msg.id;
}

void visualization::visualizeOrigin(visualization_msgs::Marker& msg, visualization_msgs::MarkerArray& marray)
{
    msg.points.clear();
    msg.ns = "origin";
    msg.points.resize(2);
    msg.points[0].x = 0;
    msg.points[0].y = 0;
    msg.points[0].z = 0;
    msg.points[1].x = 0.2;
    msg.points[1].y = 0;
    msg.points[1].z = 0;
    ++msg.id;
    msg.type = visualization_msgs::Marker::ARROW;
    msg.scale.x = 0.001;
    msg.scale.y = 0.01;
    msg.scale.z = 0.01;
    marray.markers.push_back(msg);
    msg.points[1].x = 0.0;
    msg.points[1].y = 0.2;
    msg.points[1].z = 0;
    msg.color.r = 0;
    msg.color.g = 1;
    msg.color.b = 0;
    ++msg.id;
    marray.markers.push_back(msg);
    msg.points[1].x = 0.0;
    msg.points[1].y = 0;
    msg.points[1].z = 0.2;
    msg.color.r = 0;
    msg.color.g = 0;
    msg.color.b = 1;
    ++msg.id;
    marray.markers.push_back(msg);
}

void visualization::visualizeBoundry(MeshMap& mesh, visualization_msgs::MarkerArray& msg)
{
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::MODIFY;
    m.lifetime = ros::Duration(0.2);
    m.type = visualization_msgs::Marker::POINTS;
    m.ns = mesh.frame_id_ + "_boundry_front";
    m.header.frame_id = mesh.frame_id_;
    m.points.clear();
    m.color.a = 0.8;
    m.color.r = 255.0/256.0;
    m.color.g = 128.0/256.0;
    m.color.b = 0;

    m.scale.x = 0.005;
    m.scale.y = 0.005;
    m.scale.z = 0.005;
    if(msg.markers.empty()){
        m.id = 0;
    } else{
        m.id = msg.markers.back().id +1;;
    }

    std::vector<MeshMap::VertexHandle> front = mesh.frontBoundryVertices();
    for(auto v : front){
        cslibs_math_3d::Vector3d p = mesh.getPoint(v);
        geometry_msgs::Point pg = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(p);
        m.points.push_back(pg);

    }
    msg.markers.push_back(m);
    m.ns = mesh.frame_id_ + "_boundry_back";
    ++m.id;
    m.points.clear();

    std::vector<MeshMap::VertexHandle> back = mesh.backBoundryVertices();
    for(auto v : back){
        cslibs_math_3d::Vector3d p = mesh.getPoint(v);
        geometry_msgs::Point pg = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(p);
        m.points.push_back(pg);

    }
    msg.markers.push_back(m);

}

void visualization::visualizeEdgeParticle(const EdgeParticle& p,
                                          const MeshMap& mesh,
                                          visualization_msgs::Marker& msg)
{
    cslibs_math_3d::Vector3d point = p.getPosition(mesh);
    cslibs_math_3d::Vector3d normal = p.getNormal(mesh);
    visualizeNormal(point, normal, msg);
    msg.type = visualization_msgs::Marker::ARROW;
    msg.action = visualization_msgs::Marker::MODIFY;
    msg.lifetime = ros::Duration(0.1);
    msg.header.frame_id = mesh.frame_id_;
    msg.ns = "particle";
    msg.color.a = 0.8;
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0;
    msg.scale.x = 0.005;
    msg.scale.y = 0.01;
    msg.scale.z = 0.01;
}
