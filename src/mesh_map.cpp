#include <cslibs_mesh_map/mesh_map.h>
#include <iostream>
using namespace cslibs_mesh_map;
using namespace cslibs_math_3d;

MeshMap::MeshMap() :
    calculated_edges_sum_(false),
    found_boundry_vertices_(false),
    generator_(rd_()),
    sum_edge_len_(0)
{
}

MeshMap::MeshMap(const MeshMap& other) :
    frame_id_(other.frame_id_),
    id_(other.id_),
    calculated_edges_sum_(other.calculated_edges_sum_),
    found_boundry_vertices_(other.calculated_edges_sum_),
    generator_(rd_()),
    mesh_(other.mesh_),
    sum_edge_len_(other.sum_edge_len_),
    boundry_vertices_front_(other.boundry_vertices_front_),
    boundry_vertices_back_(other.boundry_vertices_back_)
{
}

MeshMap::MeshMap(MeshMap&& other):
    frame_id_(other.frame_id_),
    id_(other.id_),
    calculated_edges_sum_(other.calculated_edges_sum_),
    found_boundry_vertices_(other.calculated_edges_sum_),
    generator_(rd_()),
    mesh_(other.mesh_),
    sum_edge_len_(other.sum_edge_len_),
    boundry_vertices_front_(other.boundry_vertices_front_),
    boundry_vertices_back_(other.boundry_vertices_back_)
{
}

MeshMap& MeshMap::operator = (const MeshMap &other)
{
    frame_id_ = other.frame_id_;
    id_ = other.id_;
    mesh_ = other.mesh_;
    sum_edge_len_ = other.sum_edge_len_;
    calculated_edges_sum_ = other.calculated_edges_sum_;
    found_boundry_vertices_ = other.found_boundry_vertices_;
    boundry_vertices_front_ = other.boundry_vertices_front_;
    boundry_vertices_back_ = other.boundry_vertices_back_;
    return *this;
}


MeshMap& MeshMap::operator = (MeshMap &&other)
{
    frame_id_ = std::move(other.frame_id_);
    id_ = std::move(other.id_);
    mesh_    = std::move(other.mesh_);
    sum_edge_len_ = std::move(other.sum_edge_len_);
    calculated_edges_sum_ = std::move(other.calculated_edges_sum_);
    found_boundry_vertices_ = std::move(other.found_boundry_vertices_);
    boundry_vertices_front_ = std::move(other.boundry_vertices_front_);
    boundry_vertices_back_ = std::move(other.boundry_vertices_back_);
    return *this;
}

bool MeshMap::loadMeshWithNormals(const std::string& file)
{
    mesh_.request_vertex_normals();
    // assure we have vertex normals
    if (!mesh_.has_vertex_normals())
    {
        std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
        return false;
    }
    OpenMesh::IO::Options opt;
    if (!OpenMesh::IO::read_mesh(mesh_, file, opt)){
        std::cerr << "read error\n";
        return false;
    }
    // If the file did not provide vertex normals, then calculate them
    if ( !opt.check( OpenMesh::IO::Options::VertexNormal ) ){
        // we need face normals to update the vertex normals
        mesh_.request_face_normals();
        // let the mesh update the normals
        mesh_.update_normals();
        // dispose the face normals, as we don't need them anymore
        mesh_.release_face_normals();
    }
    return true;
}

bool MeshMap::writeMesh(const std::string& file) const
{
    OpenMesh::IO::Options opt(OpenMesh::IO::Options::Flag::VertexNormal);
    try
    {
        if ( !OpenMesh::IO::write_mesh(mesh_, file, opt) )
        {
            std::cerr << "Cannot write mesh to file " << file << std::endl;
            return true;
        }
    }
    catch( std::exception& x )
    {
        std::cerr << x.what() << std::endl;
        return false;
    }
    return false;
}

bool MeshMap::isBoundry(const VertexIterator it) const
{
    return mesh_.is_boundary(*it);
}

bool MeshMap::isBoundry(const VertexHandle it) const
{
    return mesh_.is_boundary(it);
}

MeshMap::VertexHandle MeshMap::vertexHandle(std::size_t n)
{
    return mesh_.vertex_handle(n);
}

MeshMap::VertexHandle MeshMap::vertexHandle(VertexIterator it)
{
    return mesh_.handle(mesh_.vertex(*it));
}

MeshMap::VertexHandle MeshMap::vertexHandle(VertexOutHalfedgeIterator it)
{
    return mesh_.to_vertex_handle(*it);
}

MeshMap::VertexHandle MeshMap::toVertexHandle(EdgeIterator eit)
{
    TriMesh::HalfedgeHandle heh = mesh_.halfedge_handle(*eit,0);
    return mesh_.to_vertex_handle(heh);
}

MeshMap::VertexHandle MeshMap::fromVertexHandle(EdgeIterator eit)
{
    TriMesh::HalfedgeHandle heh = mesh_.halfedge_handle(*eit,0);
    return mesh_.from_vertex_handle(heh);
}

MeshMap::VertexHandle MeshMap::getRandomNeighbour(VertexHandle v)
{
    std::size_t n_edges = numberOfEdges(v);
    std::uniform_int_distribution<std::size_t> dist(0,n_edges);
    std::size_t index = dist(generator_);

    VertexOutHalfedgeIterator vhs =mesh_.voh_iter(v);
    for(std::size_t i = 0; i < index; ++i){
        ++vhs;
    }
    return vertexHandle(vhs);
}

MeshMap::VertexHandle MeshMap::getRandomNeighbourGreaterDistance(VertexHandle v, const cslibs_math_3d::Vector3d& start, double& dist)
{
    std::size_t n_edges = numberOfEdges(v);

    std::uniform_int_distribution<std::size_t> neighbours(0,n_edges);
    double c_dist = 0;
    VertexHandle v_n;
    VertexOutHalfedgeIterator vhs =mesh_.voh_iter(v);
    std::size_t iterations = 0;
    while(c_dist <= dist && iterations < n_edges){
        std::size_t index = neighbours(generator_);
        for(std::size_t i = 0; i < index; ++i){
            ++vhs;
        }
        v_n = vertexHandle(vhs);
        Vector3d pos = getPoint(v);
        c_dist  = (pos -start).length();
        ++iterations;
    }

    dist = c_dist;
    return v_n;
}

MeshMap::VertexIterator MeshMap::begin()
{
    return mesh_.vertices_begin();
}

const MeshMap::VertexIterator MeshMap::begin() const
{
    return mesh_.vertices_begin();
}

MeshMap::VertexIterator MeshMap::end()
{
    return mesh_.vertices_end();
}

const MeshMap::VertexIterator MeshMap::end() const
{
    return mesh_.vertices_end();
}

MeshMap::EdgeIterator MeshMap::edgeBegin()
{
    return mesh_.edges_begin();
}

MeshMap::EdgeIterator  MeshMap::edgeEnd()
{
    return mesh_.edges_end();
}
const MeshMap::EdgeIterator  MeshMap::edgeBegin() const
{
    return mesh_.edges_begin();
}
const MeshMap::EdgeIterator  MeshMap::edgeEnd() const
{
    return mesh_.edges_end();
}

std::size_t MeshMap::numberOfEdges(const VertexHandle v)
{
    std::size_t n_edges = 0;
    for(auto vohit = mesh_.voh_iter(v); vohit.is_valid(); ++vohit) {
        ++n_edges;
    }
    return n_edges;
}

std::size_t MeshMap::numberOfVertices(const VertexHandle v)
{
    std::size_t n_vert= 0;
    for(auto vvit = mesh_.vv_iter(v); vvit.is_valid(); ++vvit) {
        ++n_vert;
    }
    return n_vert;
}

void MeshMap::transform(const cslibs_math_3d::Transform3d& trans)
{
    for(VertexIterator it = begin(); it!=end(); ++it){
        Vector3d p = getPoint(it);
        Vector3d n = getNormal(it);
        p = trans * p;
        n = trans.rotation() * n;
        mesh_.set_point(*it, toPoint(p));
        mesh_.set_normal(*it, toPoint(n));
    }
}
double MeshMap::sumEdgeLength() const
{
    if(!calculated_edges_sum_){
        double edges_length = 0;
        for(auto e_it = mesh_.edges_begin(); e_it != mesh_.edges_end(); ++e_it){
            edges_length += mesh_.calc_edge_length(*e_it);
        }
        sum_edge_len_ = edges_length;
        calculated_edges_sum_ = true;
    }
    return sum_edge_len_;
}

MeshMap::VertexIterator MeshMap::getVertexCloseToPoint(const Vector3d &p) const
{
    double min_dist = std::numeric_limits<double>::infinity();
    auto it_min = mesh_.vertices_begin();
    std::size_t n_points = 0;
    TriMesh::Point pd = toPoint(p);
    for(auto it = mesh_.vertices_begin(); it!=mesh_.vertices_end(); ++it){
        TriMesh::Point pi =  mesh_.point(*it);
        TriMesh::Point diff = pi - pd;
        double d = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];
        if(d < min_dist){
            min_dist = d;
            it_min = it;
        }
        ++n_points;
    }
    std::cout << "min_dist: " << std::sqrt(min_dist) << " | #points: " << n_points << std::endl;
    return it_min;
}

cslibs_math_3d::Vector3d MeshMap::getPoint(const MeshMap::VertexIterator& it) const
{
    TriMesh::Point p = mesh_.point(*it);
    return toVector(p);
}

cslibs_math_3d::Vector3d MeshMap::getNormal(const MeshMap::VertexIterator& it) const
{
    TriMesh::Point p = mesh_.point(*it);
    return toVector(p);
}

cslibs_math_3d::Vector3d MeshMap::getPoint(const MeshMap::VertexHandle& it) const
{
    TriMesh::Point p = mesh_.point(it);
    return toVector(p);
}

cslibs_math_3d::Vector3d MeshMap::getNormal(const MeshMap::VertexHandle& it) const
{
    TriMesh::Point p = mesh_.normal(it);
    return toVector(p);
}

cslibs_math_3d::Vector3d MeshMap::toVector(const TriMesh::Point& p)
{
    return Vector3d(p[0],p[1],p[2]);
}
TriMesh::Point MeshMap::toPoint(const cslibs_math_3d::Vector3d& p)
{
    TriMesh::Point res;
    res[0] = p(0);
    res[1] = p(1);
    res[2] = p(2);
    return res;
}

MeshMap::VertexHandle MeshMap::getRandomBoundryVertexFront()
{
    seperateBoundryVertices();
    std::uniform_int_distribution<std::size_t> index(0,boundry_vertices_front_.size() -1);
    std::size_t id = index(generator_);
    return boundry_vertices_front_.at(id);

}
MeshMap::VertexHandle MeshMap::getRandomBoundryVertexEnd()
{
    seperateBoundryVertices();
    std::uniform_int_distribution<std::size_t> index(0,boundry_vertices_back_.size() -1);
    std::size_t id = index(generator_);
    return boundry_vertices_back_.at(id);
}

std::vector<MeshMap::VertexHandle> MeshMap::frontBoundryVertices()
{
    seperateBoundryVertices();
    return boundry_vertices_front_;
}

std::vector<MeshMap::VertexHandle> MeshMap::backBoundryVertices()
{
    seperateBoundryVertices();
    return boundry_vertices_back_;
}

void MeshMap::seperateBoundryVertices()
{
    if(!found_boundry_vertices_){
        std::vector<VertexHandle> boundry;
        boundry_vertices_front_.clear();
        boundry_vertices_back_.clear();
        Vector3d centroid(0,0,0);
        for(auto it = begin(); it != end(); ++it){
            VertexHandle v = vertexHandle(it);
            if(isBoundry(it) && numberOfEdges(v) != 0){
                centroid += getPoint(it);
                boundry.push_back(v);
            }
        }
        centroid /= boundry.size();
        double dc = centroid.length();
        if(dc > 1e-3){
            for(std::size_t i = 1; i < boundry.size(); ++i){
                VertexHandle v = boundry[i];
                Vector3d p = getPoint(v);
                if(p.length() < dc){
                    boundry_vertices_front_.push_back(v);
                } else{
                    boundry_vertices_back_.push_back(v);
                }
            }
        } else{
            std::vector<VertexHandle> c1;
            std::vector<VertexHandle> c2;

            Vector3d p0 = getPoint(boundry.front()) - centroid;
            for(std::size_t i = 1; i < boundry.size(); ++i){
                VertexHandle v = boundry[i];
                Vector3d p = getPoint(v) - centroid;
                double scalar = p0.dot(p);
                if(scalar > 0){
                    c1.push_back(v);
                } else {
                    c2.push_back(v);
                }
            }

            Vector3d cc1(0,0,0);
            for(auto v : c1){
                cc1 += getPoint(v);
            }
            cc1 /= c1.size();

            Vector3d cc2(0,0,0);
            for(auto v : c2){
                cc2 += getPoint(v);
            }
            cc2 /= c2.size();

            if( cc1.length() < cc2.length()){
                boundry_vertices_front_ = c1;
                boundry_vertices_back_ = c2;
            } else {
                boundry_vertices_front_ = c2;
                boundry_vertices_back_ = c1;
            }
        }
        found_boundry_vertices_ = true;

    }
}
