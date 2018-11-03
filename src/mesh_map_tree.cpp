#include <cslibs_mesh_map/mesh_map_tree.h>

using namespace cslibs_mesh_map;
using namespace cslibs_math_3d;
MeshMapTree::MeshMapTree() :
    id(0),
    set_data_(false),
    n_nodes_(0)
{

}

void MeshMapTree::setAtLeaf(const MeshMap &m, const cslibs_math_3d::Transform3d& t)
{
    if(id == 0){
        ++n_nodes_;
    }
    if(isLeaf()){
        if(!set_data_){
            map_ = m;
            transform_ = t;
            set_data_ = true;
        } else{
            MeshMapTree::Ptr mp(new MeshMapTree);
            mp->map_ = m;
            mp->transform_ = t;
            mp->id = id + 1;
            mp->set_data_ = true;
            children_.push_back(mp);
        }
    } else {
        for(MeshMapTree::Ptr c : children_){
            c->setAtLeaf(m);
        }
    }
}

void MeshMapTree::add(const std::string& parent_frame,
                      const MeshMap& m,
                      const cslibs_math_3d::Transform3d& t)
{
    if(id == 0){
        ++n_nodes_;
    }
    if(!set_data_){
        set_data_ = true;
        map_ = m;
        transform_ = t;
        parent_id_ = parent_frame;
    }
    else if(map_.frame_id_ == parent_frame){
        MeshMapTree::Ptr mp(new MeshMapTree);
        mp->map_ = m;
        mp->transform_ = t;
        mp->id = id + 1;
        mp->set_data_ = true;
        mp->parent_id_ = parent_frame;
        children_.push_back(mp);
    } else {
        for(MeshMapTree::Ptr c : children_){
            c->add(parent_frame, m, t);
        }
    }
}

bool MeshMapTree::isLeaf() const
{
    return children_.empty();
}

//MeshMap& MeshMapTree::getMap(std::string frame_id)
//{
//    if(map_.frame_id_ == frame_id){
//        return map_;
//    } else {
//        for(auto c : children_){
//            c->getMap(frame_id);
//        }
//    }
//    throw std::runtime_error("Frame_id not found");
//}

//const MeshMap& MeshMapTree::getMap(std::string frame_id) const
//{
//    if(map_.frame_id_ == frame_id){
//        return map_;
//    } else {
//        for(auto c : children_){
//            c->getMap(frame_id);
//        }
//    }
//    throw std::runtime_error("Frame_id not found");
//}


MeshMapTree *MeshMapTree::getNode(std::string frame_id)
{
//    MeshMapTree::Ptr res;
    MeshMapTree* res;
    if(map_.frame_id_ == frame_id){
//        res.reset(this);
        res = this;
        return res;
    } else {
        for(auto c : children_){
            MeshMapTree* tmp = c->getNode(frame_id);
            if(tmp){
                res = tmp;
            }
        }
    }
    return res;
}

//const MeshMapTree::Ptr MeshMapTree::getNode(std::string frame_id) const
//{
//    MeshMapTree::Ptr res;
//    if(map_.frame_id_ == frame_id){
//        res = MeshMapTree::Ptr(this);
//        return res;
//    } else {
//        for(auto c : children_){
//            MeshMapTree::Ptr tmp = c->getNode(frame_id);
//            if(tmp){
//                res = tmp;
//            }
//        }
//    }
//    return res;
//}


MeshMapTree* MeshMapTree::getNode(std::size_t map_id)
{
    MeshMapTree* res;
    if(map_.id_ == map_id){
//        res.reset(this);
        res = this;
        return res;
    } else {
        for(auto c : children_){
            MeshMapTree* tmp = c->getNode(map_id);
            if(tmp){
                res = tmp;
            }
        }
    }
    return res;
}

//const MeshMapTree& MeshMapTree::getNode(std::size_t map_id) const
//{
//    if(map_.id_ == map_id){
//        return *this;
//    } else {
//        if(children_.empty()){
//            throw std::runtime_error("map_id not found");
//        }
//        for(auto c : children_){
//            c->getNode(map_id);
//        }
//    }
//}

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
