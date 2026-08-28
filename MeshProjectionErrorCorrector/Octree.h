#pragma once

#include "macro.h"
#include <vector>
#include <cfloat>
#include <Eigen/Core>

#define OCTREE_CHILDREN_NUM 8

struct AABB {
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();

    AABB()
    {}

    AABB(Eigen::Vector3d min, Eigen::Vector3d max):
        min(min), max(max)
    {}

    bool Contains(const AABB& other) const {
        const double eps = FLT_MIN;
        return (other.min.array() + eps >= min.array()).all() &&
            (other.max.array() - eps <= max.array()).all();
    }

    bool Intersects(const AABB& other) const {
        const double eps = FLT_MIN;
        return (min.array() + eps <= other.max.array()).all() &&
            (max.array() - eps >= other.min.array()).all();
    }

    Eigen::Vector3d GetCenter() const {
        return (min + max) * 0.5f;
    }

    double Size() const {
        return (max - min).norm();
    }
};

struct OctreeObject {
    int id;
    AABB aabb;

    explicit OctreeObject(int id, const AABB& aabb = AABB());
};

class OctreeNode {

public:
    OctreeNode(const AABB& bounds, OctreeNode* parent, int depth, int maxDepth, int nodeCapacity);

    ~OctreeNode();

    bool HasChildren() const;

    bool CanSplit() const;

    OctreeObject* Find(int id);

    bool Insert(OctreeObject* obj);

    bool Remove(OctreeObject* obj);

    void Query(const AABB& aabb, std::vector<OctreeObject*>& results);

private:
    void Split();
    bool RemoveFromLocalNode(OctreeObject* obj);

public:
    AABB bounds;
    std::vector<OctreeObject*> objects;
    OctreeNode* children[OCTREE_CHILDREN_NUM];
    OctreeNode* parent;
    int depth;
    int maxDepth;
    int nodeCapacity;
};

class Octree {
public:
    Octree(const AABB& rootBounds, int maxDepth = 5, int nodeCapacity = 10);

    ~Octree();

    void Insert(int id, const AABB& bounds);
    void Update(int id, const AABB& bounds);
    bool Remove(int id);

    void Query(const AABB& aabb, std::vector<int>& results);

    void Query(const AABB& aabb, std::function<bool(int,const AABB& bounds)> callback, std::vector<int>& results);

private:
    int             maxDepth;
    int             nodeCapacity;

    OctreeNode*     root;
    std::unordered_map<int, OctreeObject*> objectMap;


};