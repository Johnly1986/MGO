#pragma once

#include "Octree.h"
#include <vector>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <Eigen/Core>

OctreeObject::OctreeObject(int id, const AABB& aabb)
    : id(id), aabb(aabb)
{
}

OctreeNode::OctreeNode(const AABB& bounds,
    OctreeNode* parent,
    int depth,
    int maxDepth,
    int nodeCapacity)
    : bounds(bounds),
    parent(parent),
    depth(depth),
    maxDepth(maxDepth),
    nodeCapacity(nodeCapacity)
{
    memset(children, 0, sizeof(children));
}

OctreeNode::~OctreeNode()
{
    for (int i = 0; i < OCTREE_CHILDREN_NUM; ++i)
    {
        if (children[i] == nullptr)
            continue;

        delete children[i];
    }
}

bool OctreeNode::HasChildren() const
{
    return children[0] != nullptr;
}

bool OctreeNode::CanSplit() const
{
    return (children[0] == nullptr && depth < maxDepth);
}

OctreeObject* OctreeNode::Find(int id)
{
    // 先检查本地节点对象
    for (auto obj : objects)
    {
        if (obj->id == id)
            return obj;
    }

    // 如果有子节点，遍历子节点查找
    if (HasChildren())
    {
        for (int i = 0; i < OCTREE_CHILDREN_NUM; ++i)
        {
            if (children[i] == nullptr)
                continue;
            OctreeObject* result = children[i]->Find(id);
            if (result != nullptr)
                return result;
        }
    }

    return nullptr;
}

bool OctreeNode::Insert(OctreeObject* obj)
{
    if (!bounds.Contains(obj->aabb))
        return false;

    if (CanSplit()) Split();

    if (HasChildren())
    {
        for (int i = 0; i < OCTREE_CHILDREN_NUM; ++i)
        {
            if (children[i]->Insert(obj))
            {
                return true;
            }
        }
    }

    objects.push_back(obj);

    //std::cout << "insert obj( " << obj->id << ", " << int(obj->aabb.Size()) << ", " << int(bounds.Size()) << " ) into " << this->depth << std::endl;
    //std::cout << "obj bounds " << obj->aabb.min.transpose() << " " << obj->aabb.max.transpose() << std::endl;
    //std::cout << "node bounds " << bounds.min.transpose() << " " << bounds.max.transpose() << "\n" << std::endl;

    return true;
}

bool OctreeNode::Remove(OctreeObject* obj)
{
    if (!obj) return false;

    // 如果有子节点，先尝试从子节点移除
    if (HasChildren())
    {
        for (int i = 0; i < OCTREE_CHILDREN_NUM; ++i)
        {
            if (children[i] == nullptr)
                continue;
            if (children[i]->bounds.Contains(obj->aabb))
            {
                if (children[i]->Remove(obj))
                {
                    return true;
                }
            }
        }
    }

    // 然后检查本地节点对象
    return RemoveFromLocalNode(obj);
}

void OctreeNode::Query(const AABB& aabb, std::vector<OctreeObject*>& results)
{
    if (!bounds.Intersects(aabb))
        return;

    for (auto obj : objects)
    {
        results.push_back(obj);
    }

    // 如果有子节点，递归查询子节点
    if (HasChildren())
    {
        for (int i = 0; i < OCTREE_CHILDREN_NUM; ++i)
        {
            if (children[i] == nullptr)
                continue;
            children[i]->Query(aabb, results);
        }
    }
}

void OctreeNode::Split()
{
    const Eigen::Vector3d center = bounds.GetCenter();
    const Eigen::Vector3d& min = bounds.min;
    const Eigen::Vector3d& max = bounds.max;

    // 创建OCTREE_CHILDREN_NUM个子节点
    for (int i = 0; i < OCTREE_CHILDREN_NUM; ++i)
    {
        Eigen::Vector3d childMin, childMax;

        // 使用位运算确定子节点位置
        childMin.x() = (i & 1) ? center.x() : min.x();
        childMax.x() = (i & 1) ? max.x() : center.x();

        childMin.y() = (i & 2) ? center.y() : min.y();
        childMax.y() = (i & 2) ? max.y() : center.y();

        childMin.z() = (i & 4) ? center.z() : min.z();
        childMax.z() = (i & 4) ? max.z() : center.z();

        // 验证子节点边界有效性
        if (childMin.x() >= childMax.x() || childMin.y() >= childMax.y() || childMin.z() >= childMax.z())
        {
            children[i] = nullptr;
        }
        else
        {
            children[i] = new OctreeNode(
                { childMin, childMax },
                this,
                depth + 1,
                maxDepth,
                nodeCapacity);
        }
    }

}

bool OctreeNode::RemoveFromLocalNode(OctreeObject* obj)
{
    auto it = std::find_if(objects.begin(), objects.end(),
        [obj](const OctreeObject* o){
            return o->id == obj->id;
        });
    if (it != objects.end())
    {
        objects.erase(it);
        return true;
    }
    return false;
}

Octree::Octree(const AABB& rootBounds, int maxDepth, int nodeCapacity)
    : root(new OctreeNode(rootBounds, nullptr, 0, maxDepth, nodeCapacity)),
    maxDepth(maxDepth),
    nodeCapacity(nodeCapacity)
{
}

Octree::~Octree()
{
    delete root;
}

void Octree::Insert(int id, const AABB& bounds)
{
    // 检查是否已存在该 id 的对象
    auto it = objectMap.find(id);
    if (it != objectMap.end())
    {
        throw std::runtime_error("Object with id " + std::to_string(id) + " already exists.");
    }

    OctreeObject* obj = new OctreeObject(id, bounds);
    bool inserted = root->Insert(obj);
    if (inserted)
    {
        objectMap[id] = obj;
    }
    else
    {
        delete obj;  // 修复内存泄漏
    }
}

bool Octree::Remove(int id)
{
    auto it = objectMap.find(id);
    if (it == objectMap.end())
    {
        return false;
    }

    if (root->Remove(it->second))
    {
        objectMap.erase(it);
        return true;
    }
    return false;
}

void Octree::Update(int id, const AABB& bounds)
{
    if (Remove(id)) Insert(id, bounds);
}

void Octree::Query(const AABB& aabb, std::vector<int>& results)
{
    std::vector<OctreeObject*> tempList;
    root->Query(aabb, tempList);
    for (auto obj : tempList)
    {
        results.push_back(obj->id);
    }
}

void Octree::Query(const AABB& aabb, std::function<bool(int, const AABB&)> callback, std::vector<int>& results)
{
    std::vector<OctreeObject*> tempList;
    root->Query(aabb, tempList);
    for (auto obj : tempList)
    {
        if (callback(obj->id, obj->aabb))
            results.push_back(obj->id);
    }
}