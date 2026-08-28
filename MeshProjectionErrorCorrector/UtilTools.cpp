#include "UtilTools.h"
#include <iostream>
#include <cfloat>
#include <assimp/scene.h>
#include <functional>

MeshTool::MeshTool()
{
}

MeshTool::~MeshTool()
{
}

aiMatrix4x4t<double> MeshTool::GetGlobalTransform(const aiNode* node)
{
    if (!node) return aiMatrix4x4t<double>();
    if (!node->mParent) return node->mTransformation;
    return GetGlobalTransform(node->mParent) * node->mTransformation;
}

aiAABB MeshTool::MergeAABB(const aiAABB& a, const aiAABB& b)
{
    return aiAABB(aiVector3D(std::min(a.mMin.x, b.mMin.x), std::min(a.mMin.y, b.mMin.y), std::min(a.mMin.z, b.mMin.z)),
        aiVector3D(std::max(a.mMax.x, b.mMax.x), std::max(a.mMax.y, b.mMax.y), std::max(a.mMax.z, b.mMax.z)));
}

aiAABB MeshTool::AbsoluteCalculateAABB(const aiScene* scene, aiNode* node)
{
    aiAABB box{aiVector3D(DBL_MAX), aiVector3D(DBL_MIN)};
    auto globalTransform = GetGlobalTransform(node);
    for (unsigned int i = 0; i < node->mNumMeshes; ++i)
    {
        aiAABB localBox = scene->mMeshes[node->mMeshes[i]]->mAABB;
        aiVector3d globalVectorA = globalTransform * aiVector3d(localBox.mMin);
        aiVector3d globalVectorB = globalTransform * aiVector3d(localBox.mMax);

        localBox.mMin.x = std::min(globalVectorA.x, globalVectorB.x);
        localBox.mMin.y = std::min(globalVectorA.y, globalVectorB.y);
        localBox.mMin.z = std::min(globalVectorA.z, globalVectorB.z);

        localBox.mMax.x = std::max(globalVectorA.x, globalVectorB.x);
        localBox.mMax.y = std::max(globalVectorA.y, globalVectorB.y);
        localBox.mMax.z = std::max(globalVectorA.z, globalVectorB.z);

        box = MergeAABB(box, localBox);

    }

    for (unsigned int i = 0; i < node->mNumChildren; ++i)
    {
        aiAABB localBox = AbsoluteCalculateAABB(scene, node->mChildren[i]);
        box.mMax.x = std::max(box.mMax.x, localBox.mMax.x);
        box.mMax.y = std::max(box.mMax.y, localBox.mMax.y);
        box.mMax.z = std::max(box.mMax.z, localBox.mMax.z);
        box.mMin.x = std::min(box.mMin.x, localBox.mMin.x);
        box.mMin.y = std::min(box.mMin.y, localBox.mMin.y);
        box.mMin.z = std::min(box.mMin.z, localBox.mMin.z);
    }
    return box;
}

aiAABB MeshTool::AbsoluteCalculateAABB(const aiScene* scene, aiMesh* mesh)
{
    aiAABB box{aiVector3D(DBL_MAX), aiVector3D(DBL_MIN)};
    //auto& globalTransform = GetGlobalTransform(node);
    return box;
}

void MeshTool::AbsoluteMoveNode(const aiScene* scene, aiNode* node, aiVector3d absPosition)
{
    if (!node || !node->mParent) return;
    auto globalParentTransform = GetGlobalTransform(node->mParent);
    aiVector3d globalMove = globalParentTransform.Inverse() * absPosition;

    node->mTransformation.a4 = globalMove.x;
    node->mTransformation.b4 = globalMove.y;
    node->mTransformation.c4 = globalMove.z;
}

void MeshTool::AbsoluteMoveVertex(const aiScene* scene, aiNode* node, std::function<aiVector3d(const aiVector3d&)> callback)
{
    if (!node || !node->mParent) return;
    aiMatrix4x4t<double> globalNodeTransform = GetGlobalTransform(node);
    aiMatrix4x4t<double> inverseGlobalNodeTransform = globalNodeTransform.Inverse();

    for (int mi = 0; mi < node->mNumMeshes; mi++)
    {
        auto& mesh = scene->mMeshes[node->mMeshes[mi]];
        for (int vi = 0; vi < mesh->mNumVertices; vi++)
        {
            aiVector3d vertex = globalNodeTransform * aiVector3d(mesh->mVertices[vi]);
            aiVector3d newvertex = inverseGlobalNodeTransform * callback(vertex);
            //std::cout << "Transform: " << mesh->mVertices[vi].x << " " << mesh->mVertices[vi].y << " " << mesh->mVertices[vi].z << " -> " << newvertex.x << " " << newvertex.y << " " << newvertex.z << std::endl;
            mesh->mVertices[vi] = aiVector3D(newvertex);
            
        }
    }
}

void MeshTool::DeleteSubtree(aiNode* node)
{
    if (!node) return;

    for (unsigned int i = 0; i < node->mNumChildren; ++i) {
        DeleteSubtree(node->mChildren[i]);
    }
    delete[] node->mChildren;
    delete[] node->mMeshes;
    node->mChildren = nullptr;
    node->mNumChildren = 0;

    node->mMeshes = nullptr;
    node->mNumMeshes = 0;

    //delete node;
}

void MeshTool::FlattenSceneNodes(const aiScene* scene)
{
    if (!scene || !scene->mRootNode) return;

    aiNode* root = scene->mRootNode;
    std::vector<aiNode*> nodesToFlatten;
    std::vector<aiMatrix4x4> globalTransforms;

    // �ݹ��ռ��ڵ㼰��ȫ�ֱ任
    std::function<void(aiNode*, const aiMatrix4x4&)> collectNodes =
        [&](aiNode* node, const aiMatrix4x4& globalTransform) {
        if (node->mParent && node->mNumMeshes > 0)
        {
            nodesToFlatten.push_back(node);
            globalTransforms.push_back(globalTransform);
        }
        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            auto& childNode = node->mChildren[i];
            collectNodes(childNode, globalTransform * childNode->mTransformation);
        }
        };

    collectNodes(root, aiMatrix4x4());

    // �����½ڵ�����
    std::vector<aiNode*> newChildren;
    for (size_t i = 0; i < nodesToFlatten.size(); ++i) {
        aiNode* original = nodesToFlatten[i];
        aiNode* newNode = new aiNode();
        newNode->mName = original->mName;
        newNode->mTransformation = globalTransforms[i];
        newNode->mNumMeshes = original->mNumMeshes;
        if (newNode->mNumMeshes > 0) {
            newNode->mMeshes = new unsigned int[newNode->mNumMeshes];
            memcpy(newNode->mMeshes, original->mMeshes, sizeof(unsigned int) * newNode->mNumMeshes);
        }
        else {
            newNode->mMeshes = nullptr;
        }
        newNode->mNumChildren = 0;
        newNode->mChildren = nullptr;
        newChildren.push_back(newNode);
    }

    DeleteSubtree(root);

    root->mNumChildren = newChildren.size();
    if (root->mNumChildren > 0) {
        root->mChildren = new aiNode * [root->mNumChildren];
        for (size_t i = 0; i < root->mNumChildren; ++i) {
            root->mChildren[i] = newChildren[i];
            root->mChildren[i]->mParent = root;
        }
    }
    else {
        root->mChildren = nullptr;
        root->mNumChildren = 0;
    }
}

aiVector3d ConvertTool::AssimpVectorTransformZY(const aiVector3d& v)
{
    return aiVector3d(v.x, v.z, v.y);
}

Eigen::Vector3d ConvertTool::CoordToEigen(const PJ_COORD& p)
{
    return Eigen::Vector3d(p.xyz.x, p.xyz.y, p.xyz.z);
}

Eigen::Vector3d ConvertTool::CoordToEigenGeo(const PJ_COORD& p)
{
    // proj_create_crs_to_crs(→EPSG:4979) without normalize_for_visualization
    // uses the CRS natural axis order (latitude, longitude). PROJ puts the
    // first axis (latitude) in lpz.lam and the second (longitude) in lpz.phi.
    // Swap back to the conventional (lon, lat, h) order used by all callers
    // and by EigenToCoordGeo (which does the inverse swap).
    return Eigen::Vector3d(p.lpz.phi, p.lpz.lam, p.lpz.z);
}

Eigen::Vector4d ConvertTool::CoordToEigen4d(const PJ_COORD& p)
{
    return Eigen::Vector4d(p.xyz.x, p.xyz.y, p.xyz.z, 1);
}

PJ_COORD ConvertTool::EigenToCoord(const Eigen::Vector3d& p)
{
    return proj_coord(p.x(), p.y(), p.z(), 0);
}

PJ_COORD ConvertTool::EigenToCoordGeo(const Eigen::Vector3d& p)
{
    // Input (lon, lat, h) → PROJ expects (lat, lon, h)
    return proj_coord(p.y(), p.x(), p.z(), 0);
}

Eigen::Vector3d ConvertTool::AssimpVectorToEigen(const aiVector3d& v)
{
    return Eigen::Vector3d(v.x, v.y, v.z);
}

aiVector3d ConvertTool::EigenToAssimpVector(const Eigen::Vector3d& v)
{
    return aiVector3D(v.x(), v.y(), v.z());
}