#pragma once
#include "macro.h"
#include <Eigen/Dense>

struct pj_ctx;
struct PJconsts;

// �����ӿں���
class MESH_PROJECTION_API IGeoreferencing
{
public:
    IGeoreferencing();
    IGeoreferencing(const std::string& srccrs);
    IGeoreferencing(const std::string& srccrs, const std::string& targetsrc);

    virtual ~IGeoreferencing();

    // Non-copyable — owns PROJ resources via raw pointers
    IGeoreferencing(const IGeoreferencing&) = delete;
    IGeoreferencing& operator=(const IGeoreferencing&) = delete;

    virtual void Solve() = 0;

    virtual Eigen::Vector3d Transform(const Eigen::Vector3d& source_position) = 0;

    /**
     * @brief Inverse: target geographic (lon_deg, lat_deg, h) -> source CRS (E, N, H)
     */
    virtual Eigen::Vector3d InverseTransform(const Eigen::Vector3d& target_position);

    /**
     * @brief Convert target CRS position to ECEF (Earth-Centered Earth-Fixed)
     */
    virtual Eigen::Vector3d TransformTargetToECEF(const Eigen::Vector3d& target_position);

    virtual Eigen::Matrix4d GCSNormal(const Eigen::Vector3d& source_position);

    /**
     * @brief Initialize PROJ CRS-to-CRS pipelines for Source→ECEF and ECEF→Target
     * @return true if both pipelines created successfully, false otherwise
     */
    bool InitPROJPipelines();

protected:
    std::string             m_srccrs;
    std::string             m_targetcrs;
    std::string             m_ecefcrs;

    pj_ctx*                 m_pj_context;
    PJconsts*               m_pj_SourceToECEF;
    PJconsts*               m_pj_ECEFToTarget;
};

