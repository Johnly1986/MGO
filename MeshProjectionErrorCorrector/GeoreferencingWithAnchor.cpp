// Copyright Johnlyon
//

#include "GeoreferencingWithAnchor.h"
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include "UtilTools.h"
#include <proj/proj.h>


GeoreferencingWithAnchor::GeoreferencingWithAnchor(const std::string& srccrs, const std::string& targetcrs)
    : IGeoreferencing(srccrs, targetcrs)
{
}

GeoreferencingWithAnchor::~GeoreferencingWithAnchor()
{
}

void GeoreferencingWithAnchor::SetParameter(const Eigen::Vector3d& anchor)
{
    m_anchor = anchor;
}

Eigen::Vector3d GeoreferencingWithAnchor::Transform(const Eigen::Vector3d& position)
{
    if (!m_pj_SourceToECEF || !m_pj_ECEFToTarget) {
        std::cerr << "Anchor Transform: PROJ pipelines not initialized" << std::endl;
        return position;
    }

    // Source CRS → ECEF
    PJ_COORD ecef = proj_trans(m_pj_SourceToECEF, PJ_FWD, ConvertTool::EigenToCoord(position));
    if (proj_errno(m_pj_SourceToECEF)) {
        std::cerr << "Anchor Transform: Source→ECEF failed" << std::endl;
        return position;
    }

    // Apply ECEF offset (source anchor → target anchor in ECEF space)
    PJ_COORD offset_ecef = proj_coord(
        ecef.xyz.x + m_ecef_offset.x(),
        ecef.xyz.y + m_ecef_offset.y(),
        ecef.xyz.z + m_ecef_offset.z(),
        0);

    // ECEF → Target CRS (EPSG:4979 geographic)
    PJ_COORD target = proj_trans(m_pj_ECEFToTarget, PJ_FWD, offset_ecef);

    return ConvertTool::CoordToEigenGeo(target);
}

Eigen::Vector3d GeoreferencingWithAnchor::InverseTransform(const Eigen::Vector3d& target_position)
{
    if (!m_pj_SourceToECEF || !m_pj_ECEFToTarget) {
        std::cerr << "Anchor InverseTransform: PROJ pipelines not initialized" << std::endl;
        return target_position;
    }

    // target CRS → ECEF (EPSG:4979 geographic input, lat-first order)
    PJ_COORD ecef = proj_trans(m_pj_ECEFToTarget, PJ_INV, ConvertTool::EigenToCoordGeo(target_position));
    if (proj_errno(m_pj_ECEFToTarget)) {
        std::cerr << "Anchor InverseTransform: Target→ECEF failed" << std::endl;
        return target_position;
    }

    // Subtract ECEF offset (reverse the forward offset)
    PJ_COORD src_ecef = proj_coord(
        ecef.xyz.x - m_ecef_offset.x(),
        ecef.xyz.y - m_ecef_offset.y(),
        ecef.xyz.z - m_ecef_offset.z(),
        0);

    // ECEF → Source CRS (reverse of m_pj_SourceToECEF which goes source→ECEF)
    PJ_COORD source = proj_trans(m_pj_SourceToECEF, PJ_INV, src_ecef);

    return ConvertTool::CoordToEigen(source);
}

void GeoreferencingWithAnchor::Solve()
{
    // Always initialize PROJ pipelines — even when both CRS are ECEF,
    // we need valid PJ objects for Transform and InverseTransform.
    if (!InitPROJPipelines()) {
        std::cerr << "Warning: PROJ pipelines not available, anchor georef disabled" << std::endl;
        m_ecef_offset = Eigen::Vector3d::Zero();
        return;
    }

    // Convert anchor from source CRS to ECEF
    PJ_COORD anchor_src_ecef = proj_trans(m_pj_SourceToECEF, PJ_FWD, ConvertTool::EigenToCoord(m_anchor));

    // Convert anchor from source CRS to target CRS (source → ECEF → target)
    PJ_COORD anchor_target = proj_trans(m_pj_ECEFToTarget, PJ_FWD, anchor_src_ecef);
    m_anchor_target = ConvertTool::CoordToEigen(anchor_target);

    // Convert anchor from target CRS back to ECEF
    // (m_pj_ECEFToTarget goes ECEF → target, so PJ_INV goes target → ECEF)
    PJ_COORD anchor_tgt_ecef = proj_trans(m_pj_ECEFToTarget, PJ_INV, anchor_target);

    // Compute ECEF offset: where the anchor should be in target's ECEF vs where it is in source's ECEF
    m_ecef_offset = Eigen::Vector3d(
        anchor_tgt_ecef.xyz.x - anchor_src_ecef.xyz.x,
        anchor_tgt_ecef.xyz.y - anchor_src_ecef.xyz.y,
        anchor_tgt_ecef.xyz.z - anchor_src_ecef.xyz.z
    );

    std::cout << std::fixed << std::setprecision(3)
        << "Anchor ECEF offset: (" << m_ecef_offset.x() << ", " << m_ecef_offset.y() << ", " << m_ecef_offset.z() << ") meters" << std::endl;
}
