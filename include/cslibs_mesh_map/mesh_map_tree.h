#ifndef MESH_MAP_TREE_H
#define MESH_MAP_TREE_H
#include <cslibs_mesh_map/mesh_map.h>
#include <cslibs_mesh_map/mesh_map_tree_node.hpp>
#include <cslibs_math_3d/linear/transform.hpp>
//#include <cslibs_mesh_map/tree.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>

namespace cslibs_mesh_map {

class EIGEN_ALIGN16 MeshMapTree
{
public:
    using Ptr = std::shared_ptr<MeshMapTree>;
    using ConstPtr = std::shared_ptr<const MeshMapTree>;
    using allocator_t = Eigen::aligned_allocator<MeshMapTree>;
    using MeshMapNodeList = std::vector<MeshMapTreeNode::Ptr>;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MeshMapTree();

    void add(const std::string& parent_frame,
             const MeshMap& m,
             const cslibs_math_3d::Transform3d& t = cslibs_math_3d::Transform3d());

    MeshMapTreeNode* getNode(std::string frame_id);
    const MeshMapTreeNode* getNode(std::string frame_id) const;

    inline std::size_t getNumberOfNodes() const {return n_nodes_;}

    MeshMapTreeNode* getNode(std::size_t map_id);
    const MeshMapTreeNode* getNode(std::size_t map_id) const;

    cslibs_math_3d::Transform3d getTranformToBase(const std::string& frame_id) const;

    void loadFromFile(const std::string& path,
                      const std::vector<std::string>& parent_ids,
                      const std::vector<std::string>& frame_ids,
                      const std::vector<std::string>& files);

    void getFrameIds(std::vector<std::string>& frame_ids) const;
    MeshMapNodeList::iterator begin();
    MeshMapNodeList::const_iterator begin() const;

    MeshMapNodeList::iterator end();
    MeshMapNodeList::const_iterator end() const;

    MeshMapTreeNode::Ptr& front();
    const MeshMapTreeNode::Ptr&  front() const;

    MeshMapTreeNode::Ptr& back();
    const MeshMapTreeNode::Ptr&  back() const;



private:
    bool set_data_;
    std::size_t n_nodes_;
    MeshMapNodeList nodes_;

};
}

#endif // MESH_MAP_TREE_H
