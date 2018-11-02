#ifndef CSLIBS_MESH_MAP_VISUALIZATION_H
#define CSLIBS_MESH_MAP_VISUALIZATION_H
#include <cslibs_mesh_map/mesh_map.h>
#include <cslibs_mesh_map/edge_particle.h>
#include <visualization_msgs/MarkerArray.h>

namespace cslibs_mesh_map {

struct visualization {

    static void visualizeVertices(const MeshMap& mesh,
                                  visualization_msgs::Marker& msg);

    static void visualizeNormal(const cslibs_math_3d::Vector3d& point,
                                const cslibs_math_3d::Vector3d& normal,
                                visualization_msgs::Marker& msg);

    static void visualizeOrigin(visualization_msgs::Marker& msg,
                                visualization_msgs::MarkerArray& marray);

    static void visualizeBoundry(const MeshMap& mesh,
                                 visualization_msgs::Marker& msg);

    static void visualizeEdgeParticle(const EdgeParticle& p,
                                      const MeshMap& mesh,
                                      visualization_msgs::Marker& msg);

};

} // namespace cslibs_mesh_map
#endif // CSLIBS_MESH_MAP_VISUALIZATION_H
