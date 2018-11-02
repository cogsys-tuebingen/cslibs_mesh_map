#ifndef TREE_HPP
#define TREE_HPP

#include <vector>
#include <memory>
#include <iostream>

template<typename T>
class TreeNode
{
public:
    typedef std::shared_ptr<TreeNode> Ptr;
    typedef std::shared_ptr<const TreeNode> ConstPtr;
public:
    TreeNode()
    {

    }

    inline bool isLeaf() const
    {
        return children_.empty();
    }

public:
    std::vector<TreeNode::Ptr> children_;
    T data_;

};


#endif // TREE_HPP
