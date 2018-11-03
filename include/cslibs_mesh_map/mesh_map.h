#ifndef MESH_MAP_H
#define MESH_MAP_H

#include <cslibs_math_3d/linear/vector.hpp>
#include <cslibs_math_3d/linear/transform.hpp>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/ArrayKernel.hh>

#include <random>

namespace cslibs_mesh_map {

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;

class MeshMap
{
public:
    using VertexHandle = TriMesh::VertexHandle;
    using VertexIterator = TriMesh::VertexIter;
    using EdgeIterator = TriMesh::EdgeIter;
    using VertexOutHalfedgeIterator = TriMesh::VertexOHalfedgeIter;
public:
    MeshMap();
    MeshMap(const MeshMap &other);
    MeshMap(MeshMap &&other);

    MeshMap& operator = (const MeshMap &other);
    MeshMap& operator = (MeshMap &&other);

    bool loadMeshWithNormals(const std::string& file);
    bool writeMesh(const std::string& file) const;
    bool isBoundry(const VertexIterator it) const;
    bool isBoundry(const VertexHandle it) const;

    VertexHandle vertexHandle(std::size_t n);
    VertexHandle vertexHandle(VertexIterator it);
    VertexHandle vertexHandle(VertexOutHalfedgeIterator it);
    VertexHandle toVertexHandle(EdgeIterator eit);
    VertexHandle fromVertexHandle(EdgeIterator eit);
    VertexHandle getRandomNeighbour(VertexHandle v);
    VertexHandle getRandomNeighbourGreaterDistance(VertexHandle v, const cslibs_math_3d::Vector3d& start, double& last_dist);
    VertexHandle getRandomBoundryVertexFront();
    VertexHandle getRandomBoundryVertexEnd();

    VertexIterator begin();
    VertexIterator end();
    const VertexIterator begin() const;
    const VertexIterator end() const;

    EdgeIterator edgeBegin();
    EdgeIterator edgeEnd();
    const EdgeIterator edgeBegin() const;
    const EdgeIterator edgeEnd() const;

    std::size_t numberOfEdges(const VertexHandle v);
    std::size_t numberOfVertices(const VertexHandle v);
    inline std::size_t numberOfEdges() const {return mesh_.n_edges();}
    inline std::size_t numberOfVertices() const  {return mesh_.n_vertices();}
    inline std::size_t numberOfFaces() const {return mesh_.n_faces();}

    std::vector<VertexHandle> frontBoundryVertices();
    std::vector<VertexHandle> backBoundryVertices();

    void transform(const cslibs_math_3d::Transform3d& trans);

    double sumEdgeLength() const;
    inline double calculateEdgeLength(const EdgeIterator eit) const {return mesh_.calc_edge_length(*eit);}

    VertexIterator getVertexCloseToPoint(const cslibs_math_3d::Vector3d& p) const;
    cslibs_math_3d::Vector3d getPoint(const MeshMap::VertexIterator& it) const;
    cslibs_math_3d::Vector3d getNormal(const MeshMap::VertexIterator& it) const;
    cslibs_math_3d::Vector3d getPoint(const MeshMap::VertexHandle& it) const;
    cslibs_math_3d::Vector3d getNormal(const MeshMap::VertexHandle& it) const;

    static cslibs_math_3d::Vector3d toVector(const TriMesh::Point& p);
    static TriMesh::Point toPoint(const cslibs_math_3d::Vector3d& p);
private:
    //Warning this method might not work properly if centroid of all boundry vertices == origin
    // Maybe replace by k-means k=2.
    void seperateBoundryVertices();
public:
    std::string frame_id_;
    std::size_t id_;
private:
    mutable bool calculated_edges_sum_;
    bool found_boundry_vertices_;
    std::random_device rd_;
    std::mt19937 generator_;
    TriMesh mesh_;
    mutable double sum_edge_len_;
    std::vector<VertexHandle> boundry_vertices_front_;
    std::vector<VertexHandle> boundry_vertices_back_;

};
} // namespace cslibs_mesh_map
#endif // MESH_MAP_H
