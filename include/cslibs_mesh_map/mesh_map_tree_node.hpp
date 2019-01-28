#ifndef MESHMAPTREENODE_HPP
#define MESHMAPTREENODE_HPP
#include <cslibs_mesh_map/mesh_map.h>
#include <cslibs_math_3d/linear/transform.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>
namespace cslibs_mesh_map {

struct EIGEN_ALIGN16 MeshMapTreeNode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<MeshMapTreeNode>;
    using Ptr = std::shared_ptr<MeshMapTreeNode>;
    using ConstPtr = std::shared_ptr<const MeshMapTreeNode>;

    inline MeshMapTreeNode():
        parent(nullptr)
    {}

    inline MeshMapTreeNode(const MeshMap& mesh, const cslibs_math_3d::Transform3d& tranform_):
        transform(tranform_),
        map(mesh),
        parent(nullptr)

    {}

    inline bool parentFrameId(std::string& frame_id) const
    {
        if(!parent){
            return false;
        }
        frame_id = parent->map.frame_id_;
        return true;
    }

    inline std::string frameId() const
    {
        return map.frame_id_;
    }

    inline void update(const cslibs_math_3d::Transform3d& t) const
    {
        transform = t;
    }

    inline std::size_t mapId() const
    {
        return map.id_;
    }

    mutable cslibs_math_3d::Transform3d transform;
    MeshMap map;
    MeshMapTreeNode* parent;
    std::vector<MeshMapTreeNode*> children;
};
}
#endif // MESHMAPTREENODE_HPP
