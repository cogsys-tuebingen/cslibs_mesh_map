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
    using FaceIterator = TriMesh::FaceIter;
    using VertexOutHalfedgeIterator = TriMesh::VertexOHalfedgeIter;
    using AdjacencyMatrix = std::map<VertexHandle, std::vector<VertexHandle>>;
public:
    MeshMap();
    MeshMap(const MeshMap &other);
    MeshMap(MeshMap &&other);

    virtual ~MeshMap();

    MeshMap& operator = (const MeshMap &other);
    MeshMap& operator = (MeshMap &&other);

    bool loadMeshWithNormals(const std::string& file, bool load_adjacency_matrix = true);
    bool writeMesh(const std::string& file) const;
    bool isBoundry(const VertexIterator it) const;
    bool isBoundry(const VertexHandle it) const;

    VertexHandle vertexHandle(const std::size_t& n);
    VertexHandle vertexHandle(VertexIterator it);
    VertexHandle vertexHandle(VertexOutHalfedgeIterator it);
    const VertexHandle vertexHandle(const std::size_t& n) const;
    const VertexHandle vertexHandle(const VertexIterator it) const;
    const VertexHandle vertexHandle(const VertexOutHalfedgeIterator it) const;
    VertexHandle toVertexHandle(EdgeIterator eit) const;
    VertexHandle fromVertexHandle(EdgeIterator eit) const;
    VertexHandle getRandomNeighbour(VertexHandle v) const;
    VertexHandle getRandomNeighbourGreaterDistance(VertexHandle v, const cslibs_math_3d::Vector3d& start, double& last_dist) const;
    VertexHandle getRandomBoundryVertexFront() const;
    VertexHandle getRandomBoundryVertexEnd() const;

    VertexIterator begin();
    VertexIterator end();
    const VertexIterator begin() const;
    const VertexIterator end() const;

    EdgeIterator edgeBegin();
    EdgeIterator edgeEnd();
    const EdgeIterator edgeBegin() const;
    const EdgeIterator edgeEnd() const;

    std::size_t numberOfEdges(const VertexHandle v) const;
    std::size_t numberOfVertices(const VertexHandle v) const;
    inline std::size_t numberOfEdges() const {return mesh_.n_edges();}
    inline std::size_t numberOfVertices() const  {return mesh_.n_vertices();}
    inline std::size_t numberOfFaces() const {return mesh_.n_faces();}

    std::vector<VertexHandle> frontBoundryVertices() const;
    std::vector<VertexHandle> backBoundryVertices() const;

    void transform(const cslibs_math_3d::Transform3d& trans);

    double sumEdgeLength() const;
    inline double calculateEdgeLength(const EdgeIterator eit) const {return mesh_.calc_edge_length(*eit);}
    inline const AdjacencyMatrix& getAdjacencyMatrix() const { return adjacency_matrix_;}
    inline const std::vector<VertexHandle>& getNeighbors(const VertexHandle& vh) const {return adjacency_matrix_.at(vh);}
    VertexIterator getVertexCloseToPoint(const cslibs_math_3d::Vector3d& p) const;
    cslibs_math_3d::Vector3d getPoint(const MeshMap::VertexIterator& it) const;
    cslibs_math_3d::Vector3d getNormal(const MeshMap::VertexIterator& it) const;
    cslibs_math_3d::Vector3d getPoint(const MeshMap::VertexHandle& it) const;
    cslibs_math_3d::Vector3d getNormal(const MeshMap::VertexHandle& it) const;

    /**
     * @brief findIntersection finds a intersection between a line and the surface, where the direction vector of the
     *  line is parallel or antiparallel to the normal of the plane.
     * @param result the resulting intersection
     * @param point supporting point of the line
     * @param normal direction vector of the line
     * @param region search region w.r.t. the supporting point
     * @return true if a intersection was found
     */
    bool findIntersection(cslibs_math_3d::Vector3d &result,
                          const cslibs_math_3d::Vector3d& point,
                          const::cslibs_math_3d::Vector3d& normal,
                          const::cslibs_math_3d::Vector3d& region = cslibs_math_3d::Vector3d(0.1,0.1,0.1));


    /**
     * @brief findIntersection finds a intersection between a line and the surface, where the direction vector of the
     *  line is parallel to the normal of the plane.
     * @param result the resulting intersection
     * @param point supporting point of the line
     * @param normal direction vector of the line
     * @param region search region w.r.t. the supporting point
     * @return true if a intersection was found
     */
    bool findParallelIntersection(cslibs_math_3d::Vector3d &result,
                                  const cslibs_math_3d::Vector3d& point,
                                  const::cslibs_math_3d::Vector3d& normal,
                                  const::cslibs_math_3d::Vector3d& region = cslibs_math_3d::Vector3d(0.1,0.1,0.1));

    /**
     * @brief findIntersection finds a intersection between a line and the surface, where the direction vector of the
     *  line is antiparallel to the normal of the plane.
     * @param result the resulting intersection
     * @param point supporting point of the line
     * @param normal direction vector of the line
     * @param region search region w.r.t. the supporting point
     * @return true if a intersection was found
     */
    bool findAntiParallelIntersection(cslibs_math_3d::Vector3d &result,
                                  const cslibs_math_3d::Vector3d& point,
                                  const::cslibs_math_3d::Vector3d& normal,
                                  const::cslibs_math_3d::Vector3d& region = cslibs_math_3d::Vector3d(0.1,0.1,0.1));

    void minimizeDistanceOrrientation(cslibs_math_3d::Vector3d &result,
                                      const cslibs_math_3d::Vector3d& point,
                                      const::cslibs_math_3d::Vector3d& normal,
                                      double scale_pos = 1.2,
                                      double scale_dir = 1.0,
                                      const::cslibs_math_3d::Vector3d& region = cslibs_math_3d::Vector3d(0.1,0.1,0.1));
    void minimizeDistanceOrrientation(cslibs_math_3d::Vector3d &pos,
                                      cslibs_math_3d::Vector3d &dir,
                                      const cslibs_math_3d::Vector3d& point,
                                      const::cslibs_math_3d::Vector3d& normal,
                                      double scale_pos = 1.2,
                                      double scale_dir = 1.0,
                                      const::cslibs_math_3d::Vector3d& region = cslibs_math_3d::Vector3d(0.1,0.1,0.1));

    bool intersect(const cslibs_math_3d::Vector3d& point,
                   const::cslibs_math_3d::Vector3d& dir, FaceIterator it,
                   cslibs_math_3d::Vector3d &result);

    static cslibs_math_3d::Vector3d toVector(const TriMesh::Point& p);
    static TriMesh::Point toPoint(const cslibs_math_3d::Vector3d& p);
private:
    //Warning this method might not work properly if centroid of all boundry vertices == origin
    // Maybe replace by k-means k=2.
    void seperateBoundryVertices() const;
    void createAdjacencyMatrix();
public:
    std::string frame_id_;
    std::size_t id_;
private:
    mutable bool calculated_edges_sum_;
    mutable bool found_boundry_vertices_;
    std::random_device rd_;
    mutable std::mt19937 generator_;
    mutable TriMesh mesh_;
    mutable double sum_edge_len_;
    mutable std::vector<VertexHandle> boundry_vertices_front_;
    mutable std::vector<VertexHandle> boundry_vertices_back_;
    AdjacencyMatrix adjacency_matrix_;

};
} // namespace cslibs_mesh_map
#endif // MESH_MAP_H
