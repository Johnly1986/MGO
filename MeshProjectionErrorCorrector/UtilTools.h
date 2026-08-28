// Copyright Johnlyon
//

#pragma once

#include "macro.h"
#include <stdexcept>
#include <assimp/matrix4x4.h>
#include <Eigen/Dense>
#include <proj/proj.h>

struct aiNode;
struct aiMesh;
struct aiScene;
struct aiAABB;

class MeshTool
{
public:

    MeshTool();

    ~MeshTool();

public:
    static aiMatrix4x4t<double> GetGlobalTransform(const aiNode* node);

    static aiAABB MergeAABB(const aiAABB& a, const aiAABB& b);

    static aiAABB AbsoluteCalculateAABB(const aiScene* scene, aiNode* node);

    static aiAABB AbsoluteCalculateAABB(const aiScene* scene, aiMesh* mesh);

    static void AbsoluteMoveNode(const aiScene* scene, aiNode* node, aiVector3d absPosition);

    static void AbsoluteMoveVertex(const aiScene* scene, aiNode* node, std::function<aiVector3d(const aiVector3d&)> callback);

    static void DeleteSubtree(aiNode* node);

    static void FlattenSceneNodes(const aiScene* scene);

};

class ConvertTool
{
public:
    static aiVector3d AssimpVectorTransformZY(const aiVector3d& v);

    /// Convert PROJ coord → Eigen (xyz / ECEF-projected, no axis swap)
    static Eigen::Vector3d CoordToEigen(const PJ_COORD& p);

    /// Convert PROJ coord → Eigen for geographic CRS (EPSG:4979 lat-first → lon,lat,h)
    static Eigen::Vector3d CoordToEigenGeo(const PJ_COORD& p);

    static Eigen::Vector4d CoordToEigen4d(const PJ_COORD& p);

    /// Convert Eigen → PROJ coord (xyz / ECEF-projected, no axis swap)
    static PJ_COORD EigenToCoord(const Eigen::Vector3d& p);

    /// Convert Eigen (lon,lat,h) → PROJ coord for EPSG:4979 lat-first axis order
    static PJ_COORD EigenToCoordGeo(const Eigen::Vector3d& p);

    static Eigen::Vector3d AssimpVectorToEigen(const aiVector3d& v);

    static aiVector3d EigenToAssimpVector(const Eigen::Vector3d& v);
};