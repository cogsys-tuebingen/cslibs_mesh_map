#ifndef EDGE_PARTICLE_H
#define EDGE_PARTICLE_H
#include <cslibs_mesh_map/mesh_map.h>
//#include <cslibs_math_3d/linear/vector.hpp>
#include <tf/tf.h>
namespace cslibs_mesh_map {

struct EIGEN_ALIGN16 EdgeParticle
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<EdgeParticle>;
    using Ptr         = std::shared_ptr<EdgeParticle>;

    EdgeParticle();
    void setVertices(MeshMap& map,
                     MeshMap::VertexIterator active_vertex,
                     MeshMap::VertexOutHalfedgeIterator goal_vertex);

    void setVertices(MeshMap& map,
                     std::size_t active_vertex,
                     MeshMap::VertexOutHalfedgeIterator goal_vertex);

    void setVertices(MeshMap& map,
                     std::size_t active_vertex);

    void setVertices(MeshMap& map,
                     MeshMap::VertexHandle active_vertex);

    cslibs_math_3d::Vector3d getPosition(const MeshMap& map) const;
    cslibs_math_3d::Vector3d getActiveVertex(const MeshMap& map) const;
    cslibs_math_3d::Vector3d getGoalVertex(const MeshMap& map) const;
    void updateEdgeLength(const MeshMap& map);

    double getDistanceToGoal() const;

    double getDistanceFromStart() const;

    cslibs_math_3d::Vector3d  getNormal(const MeshMap &map) const;

    MeshMap::VertexHandle active_vertex;
    MeshMap::VertexHandle goal_vertex;
    double s;
    double e;
    double weight;
    std::size_t map_id;
};
} // namespace cslibs_mesh_map

#endif // EDGE_PARTICLE_H
