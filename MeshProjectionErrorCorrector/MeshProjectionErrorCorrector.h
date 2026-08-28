// Copyright Johnlyon
//

#pragma once

#include "macro.h"
#include <string>
#include <vector>
#include <map>
#include <stdexcept>

struct pj_ctx;
struct PJconsts;
struct aiScene;
class IGeoreferencing;

class MESH_PROJECTION_API CMeshProjectionErrorCorrector
{
public:
    /**
     * @brief 构造函数，初始化 CMeshProjectionErrorCorrector 对象。
     */
    CMeshProjectionErrorCorrector();

    /**
     * @brief 析构函数，释放 CMeshProjectionErrorCorrector 对象占用的资源。
     */
    ~CMeshProjectionErrorCorrector();

public:
    void Transform(IGeoreferencing* georeferencing, const aiScene* scene, double offsetX, double offsetY, double offsetZ = 0, bool isZUp = false);

private:
    aiScene*            m_scene;
};
