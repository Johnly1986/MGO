// Copyright Johnlyon
//

#include "MeshProjectionErrorCorrector.h"
#include "UtilTools.h"
#include "CoordinateTransform.hpp"
#include "Log.hpp"
#include "IGeoreferencing.h"
using namespace MGO;
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <boost/regex.hpp>
#include <iomanip>
#include <iostream>
#include <proj/proj.h>
#include "Octree.h"

CMeshProjectionErrorCorrector::CMeshProjectionErrorCorrector()
{
    m_scene = nullptr;
}

CMeshProjectionErrorCorrector::~CMeshProjectionErrorCorrector()
{

}

void InitTree(const aiScene* scene, Octree* tree)
{
    for (int ni = 0; ni < scene->mRootNode->mNumChildren; ni++)
    {
        auto& child = scene->mRootNode->mChildren[ni];
        auto box = MeshTool::AbsoluteCalculateAABB(scene, child);
        aiVector3d min = box.mMin;
        aiVector3d max = box.mMax;
        tree->Insert(ni, AABB(Eigen::Vector3d(min.x, min.y, min.z), Eigen::Vector3d(max.x, max.y, max.z)));
    }
}

void CMeshProjectionErrorCorrector::Transform(IGeoreferencing* georeferencing, const aiScene* scene, double offsetX, double offsetY, double offsetZ, bool isZUp)
{
    // 模型投影相对位置变换
    MeshTool::FlattenSceneNodes(scene);

    // Select the source coordinate frame based on the model's up-axis.
    // ZUp: X=East, Y=North, Z=Up   (pre-rotated models, e.g. M1 roadbed)
    // AssimpYUp: X=East, Y=Up, Z=South  (default Assimp import)
    CoordinateFrame sourceFrame = isZUp ? CoordinateFrame::ZUp : CoordinateFrame::AssimpYUp;
    MGO_LOG(Info) << "Projection source frame: " << (isZUp ? "ZUp" : "AssimpYUp");

    aiVector3d offset = aiVector3d(offsetX, offsetY, offsetZ);

    aiAABB sceneBox = MeshTool::AbsoluteCalculateAABB(scene, scene->mRootNode);

    aiVector3d graphicsCenter = offset;
    Eigen::Vector3d center(graphicsCenter.x, graphicsCenter.z, graphicsCenter.y);

    Eigen::Matrix4d toCGSTransform = georeferencing->GCSNormal(center);
    MGO_LOG(Info) << "GCSNormal matrix:\n" << std::fixed << std::setprecision(14) << toCGSTransform.transpose();

    Eigen::Matrix4d moveMatrix = Eigen::Matrix4d::Identity();
    moveMatrix.block<3, 1>(0, 3) = -center;
    Eigen::Matrix4d toEarthTransform = toCGSTransform * moveMatrix;
    toEarthTransform = toEarthTransform.inverse();

    for (int i = 0; i < scene->mRootNode->mNumChildren; i++)
    {
        auto& child = scene->mRootNode->mChildren[i];
        MeshTool::AbsoluteMoveVertex(scene, child, [&](const aiVector3d& absPosition) {
            aiVector3d rawPos = absPosition + offset;
            double in[3] = {rawPos.x, rawPos.y, rawPos.z};
            double out_proj[3];
            CoordinateTransform::Convert(in, sourceFrame, out_proj, CoordinateFrame::Projected);
            Eigen::Vector3d offsetPosition(out_proj[0], out_proj[1], out_proj[2]);
            Eigen::Vector3d ecefVector3d = georeferencing->TransformTargetToECEF(georeferencing->Transform(offsetPosition));
            Eigen::Vector4d projed_center = toEarthTransform * Eigen::Vector4d(ecefVector3d.x(), ecefVector3d.y(), ecefVector3d.z(), 1);

            Eigen::Vector3d proj3 = projed_center.head<3>() / projed_center.w();
            double proj_in[3] = {proj3.x(), proj3.y(), proj3.z()};
            double ayu_out[3];
            CoordinateTransform::Convert(proj_in, CoordinateFrame::Projected,
                                          ayu_out, sourceFrame);
            aiVector3d newPosition(ayu_out[0] - offset.x, ayu_out[1] - offset.y, ayu_out[2] - offset.z);

            //std::cout << std::fixed << std::setprecision(6) << "Transform: " << absPosition.x << " " << absPosition.y << " " << absPosition.z << " -> " << newPosition.x << " " << newPosition.y << " " << newPosition.z << std::endl;
            return newPosition;
            });
    }
}
