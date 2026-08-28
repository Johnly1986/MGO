#pragma once
#include "IGeoreferencing.h"
#include <proj/proj.h>

struct MESH_PROJECTION_API SevenParameter
{
    SevenParameter()
        : isCoordinateFrame(false)
        , mx(0), my(0), mz(0), rx(0), ry(0), rz(0), scale(0)
    {
    }

    SevenParameter(double in_mx, double in_my, double in_mz, double in_rx, double in_ry, double in_rz, double in_scale)
        : isCoordinateFrame(false)
        , mx(in_mx), my(in_my), mz(in_mz), rx(in_rx), ry(in_ry), rz(in_rz), scale(in_scale)
    {
    }

    double& operator[](size_t index)
    {
        switch (index)
        {
        case 0: return mx;
        case 1: return my;
        case 2: return mz;
        case 3: return rx;
        case 4: return ry;
        case 5: return rz;
        case 6: return scale;
        default:
#if defined(_DEBUG) || defined(DEBUG)
            assert(false && "Index out of bounds");
#endif
            throw std::out_of_range("SevenParameter index out of range");
        }
    }

    double operator[](size_t index) const
    {
        return const_cast<SevenParameter*>(this)->operator[](index);
    }

    bool isCoordinateFrame;
    double mx, my, mz, rx, ry, rz, scale;
};

class MESH_PROJECTION_API GeoreferencingWith7Parameters : public IGeoreferencing
{
public:
    GeoreferencingWith7Parameters(const std::string& srccrs = "", const std::string& targetcrs = "");

    ~GeoreferencingWith7Parameters();

public:
    void SetParameter(const SevenParameter& parameter);

public:
    void Solve() override;

    Eigen::Vector3d Transform(const Eigen::Vector3d& position) override;
    Eigen::Vector3d InverseTransform(const Eigen::Vector3d& target_position) override;

protected:
    PJ*                 m_pj_transform;
    SevenParameter      m_parameter;

    // Manual TM inverse projection fallback (when PROJ fails on custom ellipsoids)
    bool                m_useProj;
    bool                m_pj_returns_degrees{false};  // CRS-aware (true) vs raw Conversion (false)
    bool                m_proj_warned{false};          // per-instance fallback warning
    double              m_tm_lon0;
    double              m_tm_lat0;
    double              m_tm_k0;
    double              m_tm_a;
    double              m_tm_rf;
    double              m_tm_fe;
    double              m_tm_fn;
};
