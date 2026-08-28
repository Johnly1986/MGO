#pragma once
#include "IGeoreferencing.h"

class MESH_PROJECTION_API GeoreferencingWithAnchor : public IGeoreferencing
{
public:
    GeoreferencingWithAnchor(const std::string& srccrs = "", const std::string& targetcrs = "");

    ~GeoreferencingWithAnchor();

public:
    void SetParameter(const Eigen::Vector3d& anchor);

public:
    void Solve() override;

    Eigen::Vector3d Transform(const Eigen::Vector3d& position) override;
    Eigen::Vector3d InverseTransform(const Eigen::Vector3d& target_position) override;

protected:
    Eigen::Vector3d     m_anchor;
    Eigen::Vector3d     m_anchor_target;    // anchor position in target CRS
    Eigen::Vector3d     m_ecef_offset;      // offset in ECEF space (source→target)
};
