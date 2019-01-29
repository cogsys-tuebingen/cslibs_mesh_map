#include <cslibs_mesh_map/mesh_map_tree.h>

using namespace cslibs_mesh_map;
using namespace cslibs_math_3d;

MeshMapTree::MeshMapTree() :
    set_data_(false),
    n_nodes_(0)
{

}

void MeshMapTree::add(const std::string& parent_frame,
                      const MeshMap& m,
                      const cslibs_math_3d::Transform3d& t)
{
    MeshMapTreeNode::Ptr node(new MeshMapTreeNode(m,t));
    nodes_.emplace_back(node);
    for(std::size_t i = 0; i < n_nodes_; ++i){
        MeshMapTreeNode::Ptr n = nodes_[i];
        if(n->frameId() == parent_frame){
            node->parent = n.get();
            n->children.push_back(node.get());
            break;
        }
    }
    ++n_nodes_;
}

MeshMapTreeNode* MeshMapTree::getNode(std::string frame_id)
{
    MeshMapTreeNode* res = nullptr;
    for(MeshMapTreeNode::Ptr n : nodes_){
        if(n->frameId() == frame_id){
            res = n.get();
            break;
        }
    }
    return res;
}

const MeshMapTreeNode* MeshMapTree::getNode(std::string frame_id) const
{
    const MeshMapTreeNode* res = nullptr;
    for(const MeshMapTreeNode::Ptr n : nodes_){
        if(n->frameId() == frame_id){
            res = n.get();
            break;
        }
    }
    return res;
}


MeshMapTreeNode* MeshMapTree::getNode(std::size_t map_id)
{
    return nodes_.at(map_id).get();
}

const MeshMapTreeNode* MeshMapTree::getNode(std::size_t map_id) const
{
   return nodes_.at(map_id).get();
}

void MeshMapTree::loadFromFile(const std::string& path,
                               const std::vector<std::string>& parent_ids,
                               const std::vector<std::string>& frame_ids,
                               const std::vector<std::string>& files)
{
    if((frame_ids.size() != files.size() && frame_ids.size() == parent_ids.size()) || frame_ids.empty()){
        throw std::runtime_error("For each mesh a frame_id is required!");
    }
    for(std::size_t i = 0; i < frame_ids.size(); ++i){
        std::string file = path + "/" + files[i];
        MeshMap link;
        if(!link.loadMeshWithNormals(file)){
            throw std::runtime_error("Invalid file!");
        }
        link.frame_id_ = frame_ids[i];
        link.id_ = i;

        add(parent_ids[i], link);
    }
}

void MeshMapTree::getFrameIds(std::vector<std::string>& frame_ids) const
{
    for (const MeshMapTreeNode::Ptr& c : nodes_){
        frame_ids.push_back(c->frameId());
    }
}

cslibs_math_3d::Transform3d MeshMapTree::getTranformToBase(const std::string& frame_id) const
{
    cslibs_math_3d::Transform3d transform;
    transform.identity();
    MeshMapTreeNode const* target  = getNode(frame_id);
    if(!target){
        throw std::runtime_error("frame_id " + frame_id + "not found!");
    }
    while(target){
        transform =  target->transform * transform;
        target = target->parent;
    }
    return transform;
}

MeshMapTree::MeshMapNodeList::iterator MeshMapTree::begin()
{
    return nodes_.begin();
}

MeshMapTree::MeshMapNodeList::const_iterator MeshMapTree::begin() const
{
    return nodes_.begin();
}

MeshMapTree::MeshMapNodeList::iterator MeshMapTree::end()
{
    return nodes_.end();
}

MeshMapTree::MeshMapNodeList::const_iterator MeshMapTree::end() const
{
    return nodes_.end();
}

MeshMapTreeNode::Ptr& MeshMapTree::front()
{
    return nodes_.front();
}

const MeshMapTreeNode::Ptr&  MeshMapTree::front() const
{
    return nodes_.front();
}

MeshMapTreeNode::Ptr& MeshMapTree::back()
{
    return nodes_.back();
}

const MeshMapTreeNode::Ptr&  MeshMapTree::back() const
{
    return nodes_.back();
}
