// Copyright Johnlyon
//

#pragma once

#include "macro.h"
#include <string>
#include <vector>
#include <map>
#include <stdexcept>

struct aiMesh;
struct aiScene;
namespace Assimp
{
    class Importer;
}

struct Vector3 {
    float x;
    float y;
    float z;

    Vector3()
        : x(0.0f), y(0.0f), z(0.0f)
    {
    }

    Vector3(float x, float y, float z)
        : x(x), y(y), z(z)
    {
    }
};

struct Vector3D {
    double x;
    double y;
    double z;

    Vector3D()
        : x(0.0f), y(0.0f), z(0.0f)
    {
    }

    Vector3D(double x, double y, double z)
        : x(x), y(y), z(z)
    {
    }
};

struct Vector2 {
    float x;
    float y;
    
    Vector2()
        : x(0.0f), y(0.0f)
    {
    }

    Vector2(float x, float y)
        : x(x), y(y)
    {
    }
};

struct Vector2D {
    double x;
    double y;

    Vector2D()
        : x(0.0f), y(0.0f)
    {
    }

    Vector2D(double x, double y)
        : x(x), y(y)
    {
    }
};

struct OptimizerItem
{
    std::string name;           // 匹配项
    float       error;          // 误差
    float       nweight;        // 法线权重
    float       threshold;      // 简化零界点
    bool        lockBorder;     // 固定边缘
    bool        localError;     // 是否使用全局误差

    OptimizerItem()
    {
        error = 0.0f;
        nweight = 0.0f;
        threshold = 0.0f;
        lockBorder = false;
        localError = false;
    }

    OptimizerItem(const std::string& name, float error, float nweight, float threshold, bool lockborder, bool localerror)
    {
        this->name = name;
        this->error = error;
        this->nweight = nweight;
        this->threshold = threshold;
        this->lockBorder = lockborder;
        this->localError = localerror;
    }
};

struct OptimizerConfig
{
    bool reorder;
    float scale = 1;
    Vector3D move;
    Vector3D rotation;
    
    std::vector<OptimizerItem> items;
};

class MESH_GROUP_OPTIMIZER_API CMeshGroupOptimizer
{
public:
    /**
     * @brief 构造函数，初始化 CMeshGroupOptimizer 对象。
     */
    CMeshGroupOptimizer();

    /**
     * @brief 析构函数，释放 CMeshGroupOptimizer 对象占用的资源。
     */
    ~CMeshGroupOptimizer();

    /**
     * @brief 加载网格数据。
     * @param filename 输入文件名，指定要加载的网格组文件路径。
     * @return 操作成功返回 true，否则返回 false。
     */
    const aiScene* Load(const std::string& filename, bool rebuild = false);

    /**
     * @brief 执行网格组优化操作。
     * @return 操作成功返回 true，否则返回 false。
     */
    bool Optimize(const OptimizerConfig& config);

    /**
     * @brief 保存优化后的网格数据。
     * @param filename 输出文件名，指定保存优化结果的文件路径。
     * @return 操作成功返回 true，否则返回 false。
     */
    bool Save(const std::string& filename, bool isConvertLeftHand = false);

    /**
     * @brief 静态方法：对外部已加载的场景执行网格简化（in-place）。
     *        不需要 Load()，直接操作传入的 aiScene。
     * @param scene 已加载的 Assimp 场景（网格将被原地修改）
     * @param config 简化配置
     * @return 操作成功返回 true，否则返回 false。
     */
    static bool SimplifyScene(const aiScene* scene, const OptimizerConfig& config);

private:
    static bool GetMatchedOptimizerItem(const std::string& name, const OptimizerConfig& config, OptimizerItem& optimizerItem);
    static unsigned int GetSimplificationOptions(const OptimizerItem& optimizerItem);
    static size_t MergeVertices(std::vector<unsigned int>& indices, std::vector<Vector3>& vertices, std::vector<Vector3>& normals, std::vector<Vector2>& uvs);
    static size_t KeepEffectiveVertices(std::vector<unsigned int>& indices, std::vector<Vector3>& vertices, std::vector<Vector3>& normals, std::vector<Vector2>& uvs);
    static bool PerformOptimization(aiMesh* mesh, const OptimizerItem& optimizerItem, bool reorder);

private:
    Assimp::Importer*   importer;
    bool                m_rebuild;
    const aiScene*      m_scene;                // 存储加载的网格数据
};
