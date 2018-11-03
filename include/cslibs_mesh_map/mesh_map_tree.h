#ifndef MESH_MAP_TREE_H
#define MESH_MAP_TREE_H
#include <cslibs_mesh_map/mesh_map.h>
#include <cslibs_math_3d/linear/transform.hpp>
//#include <cslibs_mesh_map/tree.hpp>
#include <vector>
#include <memory>

namespace cslibs_mesh_map {

class EIGEN_ALIGN16 MeshMapTree{
public:
    typedef std::shared_ptr<MeshMapTree> Ptr;
    typedef std::shared_ptr<const MeshMapTree> ConstPtr;
public:
    MeshMapTree();

    void setAtLeaf(const MeshMap& m ,
                   const cslibs_math_3d::Transform3d& t= cslibs_math_3d::Transform3d());
    void add(const std::string& parent_frame,
             const MeshMap& m,
             const cslibs_math_3d::Transform3d& t = cslibs_math_3d::Transform3d());
    bool isLeaf() const;

//    MeshMap& getMap(std::string frame_id);
//    const MeshMap& getMap(std::string frame_id) const;

    MeshMapTree::Ptr getNode(std::string frame_id);
//    const MeshMapTree::Ptr getNode(std::string frame_id) const;

    inline std::size_t getNumberOfNodes() const {return n_nodes_;}

    MeshMapTree::Ptr getNode(std::size_t map_id);
//    MeshMapTree::ConstPtr getNode(std::size_t map_id) const;

    void loadFromFile(const std::string& path,
                      const std::vector<std::string>& parent_ids,
                      const std::vector<std::string>& frame_ids,
                      const std::vector<std::string>& files);


    std::string parent_id_;
    cslibs_math_3d::Transform3d transform_;
    MeshMap map_;
    std::size_t id;
    std::vector<MeshMapTree::Ptr, cslibs_math_3d::Transform3d::allocator_t> children_;
private:
    bool set_data_;
    std::size_t n_nodes_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

#endif // MESH_MAP_TREE_H
