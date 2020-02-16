#include <cslibs_mesh_map/edge_particle.h>
#include <cslibs_math_3d/linear/quaternion.hpp>
using namespace cslibs_mesh_map;
using namespace cslibs_math_3d;

EdgeParticle::EdgeParticle():
    s(0),
    e(0),
    map_id(0)
{
}

EdgeParticle::EdgeParticle(const EdgeParticle &other) :
    active_vertex(other.active_vertex),
    goal_vertex(other.goal_vertex),
    s(other.s),
    e(other.e),
    map_id(other.map_id),
    last_update(other.last_update),
    force(other.force),
    phi(other.phi),
    theta(other.theta)
{
}

EdgeParticle::EdgeParticle(EdgeParticle &&other) :
    active_vertex(other.active_vertex),
    goal_vertex(other.goal_vertex),
    s(other.s),
    e(other.e),
    map_id(other.map_id),
    last_update(other.last_update),
    force(other.force),
    phi(other.phi),
    theta(other.theta)
{
}

void EdgeParticle::setVertices(MeshMap& map,
                               MeshMap::VertexIterator active_vertex,
                               MeshMap::VertexOutHalfedgeIterator goal_vertex)
{
    this->active_vertex = map.vertexHandle(active_vertex);
    this->goal_vertex = map.vertexHandle(goal_vertex);
    updateEdgeLength(map);
}

void EdgeParticle::setVertices(MeshMap& map,
                               std::size_t active_vertex,
                               MeshMap::VertexOutHalfedgeIterator goal_vertex)
{
    this->active_vertex = map.vertexHandle(active_vertex);
    this->goal_vertex = map.vertexHandle(goal_vertex);
    updateEdgeLength(map);
}


void EdgeParticle::setVertices(MeshMap& map,
                               std::size_t active_vertex)
{
    this->active_vertex = map.vertexHandle(active_vertex);
    this->goal_vertex = map.getRandomNeighbour(this->active_vertex);
    updateEdgeLength(map);
}

void EdgeParticle::setVertices(MeshMap& map,
                               MeshMap::VertexHandle active_vertex)
{
    this->active_vertex = active_vertex;
    this->goal_vertex = map.getRandomNeighbour(this->active_vertex);
    updateEdgeLength(map);
}

cslibs_math_3d::Vector3d EdgeParticle::getPosition(const MeshMap& map) const
{
    Vector3d p0 = map.getPoint(active_vertex);
    Vector3d p1 = map.getPoint(goal_vertex);
    Vector3d pos = p0 + (p1 - p0) * s;
    return pos;
}

cslibs_math_3d::Vector3d EdgeParticle::getActiveVertex(const MeshMap& map) const
{
    return map.getPoint(active_vertex);
}

cslibs_math_3d::Vector3d EdgeParticle::getGoalVertex(const MeshMap& map) const
{
    return map.getPoint(goal_vertex);
}

void EdgeParticle::updateEdgeLength(const MeshMap& map)
{
    Vector3d s = getActiveVertex(map);
    Vector3d g = getGoalVertex(map);
    e = (g - s).length();
}

double EdgeParticle::getDistanceToGoal() const
{
    return (1-s) * e;
}

double EdgeParticle::getDistanceFromStart() const
{
    return s * e;
}


cslibs_math_3d::Vector3d EdgeParticle::getNormal(const MeshMap &map) const
{
    return map.getNormal(active_vertex);
}

cslibs_math_3d::Vector3d EdgeParticle::getDirection(const MeshMap &map) const
{
    cslibs_math_3d::Vector3d normal = map.getNormal(active_vertex);
    cslibs_math_3d::Vector3d z(0,0,1);
    cslibs_math_3d::Vector3d  axis = z.cross(normal);
    double alpha = std::acos(z.dot(normal));
    cslibs_math_3d::Vector3d dir_local(-std::sin(theta)*std::cos(phi),
                                       -std::sin(theta)*std::sin(phi),
                                       -std::cos(theta));
    cslibs_math_3d::Quaterniond q(alpha, axis);
    return q*dir_local;

}
