// test_coordframe — verify CoordinateTransform produces identical output
// to the old scattered implementations (AxisMapper, ConvertTool, TileBuilder).
//
// Every test runs the SAME input through BOTH old and new, and asserts
// the outputs are bit-identical or within floating-point tolerance.

#include "CoordinateTransform.hpp"
#include "AxisMapper.h"
#include "UtilTools.h"
#include "GeodeticMath.h"

#include <iostream>
#include <cmath>
#include <cassert>
#include <cstring>

using namespace MGO;

static int g_passed = 0, g_failed = 0;
#define TEST(n) do { std::cerr << "  " << n << "... "; } while(0)
#define PASS()  do { std::cerr << "PASS" << std::endl; ++g_passed; } while(0)
#define FAIL(m) do { std::cerr << "FAIL: " << m << std::endl; ++g_failed; } while(0)

static const double EPS = 1e-14;

// ---------------------------------------------------------------------------
// V1: Convert(AssimpYUp, ENU) == AxisMapper::AssimpToENU
// ---------------------------------------------------------------------------
static void test_assimpYUp_to_enu()
{
    double test_pts[][3] = {
        {0,0,0}, {1,0,0}, {0,1,0}, {0,0,1},
        {100.5, -20.3, 45.7}, {-1e6, 5e6, -3e6},
    };
    for (auto& p : test_pts) {
        // Old
        double old_e=0, old_n=0, old_u=0;
        AxisMapper::AssimpToENU(p[0], p[1], p[2], old_e, old_n, old_u);

        // New
        double new_out[3];
        CoordinateTransform::Convert(p, CoordinateFrame::AssimpYUp,
                                     new_out, CoordinateFrame::ENU);

        if (std::abs(old_e-new_out[0]) > EPS ||
            std::abs(old_n-new_out[1]) > EPS ||
            std::abs(old_u-new_out[2]) > EPS) {
            FAIL("mismatch at ("<<p[0]<<","<<p[1]<<","<<p[2]<<") old=("
                 <<old_e<<","<<old_n<<","<<old_u<<") new=("
                 <<new_out[0]<<","<<new_out[1]<<","<<new_out[2]<<")");
            return;
        }
    }
    PASS();
}

// ---------------------------------------------------------------------------
// V2: Convert(ENU, AssimpYUp) == AxisMapper::ENUToAssimp
// ---------------------------------------------------------------------------
static void test_enu_to_assimpYUp()
{
    double test_pts[][3] = {
        {0,0,0}, {1,0,0}, {0,1,0}, {0,0,1},
        {100.5, -20.3, 45.7}, {-1e6, 5e6, -3e6},
    };
    for (auto& p : test_pts) {
        double old_x=0, old_y=0, old_z=0;
        AxisMapper::ENUToAssimp(p[0], p[1], p[2], old_x, old_y, old_z);

        double new_out[3];
        CoordinateTransform::Convert(p, CoordinateFrame::ENU,
                                     new_out, CoordinateFrame::AssimpYUp);

        if (std::abs(old_x-new_out[0]) > EPS ||
            std::abs(old_y-new_out[1]) > EPS ||
            std::abs(old_z-new_out[2]) > EPS) {
            FAIL("mismatch");
            return;
        }
    }
    PASS();
}

// ---------------------------------------------------------------------------
// V3: Convert(AssimpYUp, Projected) == ConvertTool::AssimpVectorTransformZY
// ---------------------------------------------------------------------------
static void test_assimpYUp_to_projected()
{
    double test_pts[][3] = {
        {0,0,0}, {1,2,3}, {-5,10,-15}, {1e6,2e6,3e6},
    };
    for (auto& p : test_pts) {
        aiVector3d ai_in(p[0], p[1], p[2]);
        aiVector3d old_ai = ConvertTool::AssimpVectorTransformZY(ai_in);

        double new_out[3];
        CoordinateTransform::Convert(p, CoordinateFrame::AssimpYUp,
                                     new_out, CoordinateFrame::Projected);

        if (std::abs(old_ai.x-new_out[0]) > EPS ||
            std::abs(old_ai.y-new_out[1]) > EPS ||
            std::abs(old_ai.z-new_out[2]) > EPS) {
            FAIL("mismatch at ("<<p[0]<<","<<p[1]<<","<<p[2]<<") old=("
                 <<old_ai.x<<","<<old_ai.y<<","<<old_ai.z<<") new=("
                 <<new_out[0]<<","<<new_out[1]<<","<<new_out[2]<<")");
            return;
        }
    }
    PASS();
}

// ---------------------------------------------------------------------------
// V4: Convert(ZUp, AssimpYUp) == TileBuilder std::swap(y,z)
// ---------------------------------------------------------------------------
static void test_zUp_to_assimpYUp()
{
    double test_pts[][3] = {
        {0,0,0}, {1,2,3}, {-5,10,-15}, {1e6,2e6,3e6},
    };
    for (auto& p : test_pts) {
        double old_x = p[0], old_y = p[1], old_z = p[2];
        std::swap(old_y, old_z);  // TileBuilder convention

        double new_out[3];
        CoordinateTransform::Convert(p, CoordinateFrame::ZUp,
                                     new_out, CoordinateFrame::AssimpYUp);

        if (std::abs(old_x-new_out[0]) > EPS ||
            std::abs(old_y-new_out[1]) > EPS ||
            std::abs(old_z-new_out[2]) > EPS) {
            FAIL("mismatch");
            return;
        }
    }
    PASS();
}

// ---------------------------------------------------------------------------
// V5: Convert(AssimpYUp, TilesZUp) bbox == AxisMapper::BBoxAssimpToTilesZUp
// ---------------------------------------------------------------------------
static void test_bbox_assimpYUp_to_tilesZUp()
{
    double bmin[3] = { -10, 0, -20 };
    double bmax[3] = {  10, 5,  20 };

    double old_omin[3], old_omax[3];
    AxisMapper::BBoxAssimpToTilesZUp(bmin, bmax, old_omin, old_omax);

    double new_omin[3], new_omax[3];
    CoordinateTransform::ConvertBBox(bmin, bmax, CoordinateFrame::AssimpYUp,
                                     new_omin, new_omax, CoordinateFrame::TilesZUp);

    for (int i = 0; i < 3; ++i) {
        if (std::abs(old_omin[i]-new_omin[i]) > EPS ||
            std::abs(old_omax[i]-new_omax[i]) > EPS) {
            FAIL("mismatch at axis " << i);
            return;
        }
    }
    PASS();
}

// ---------------------------------------------------------------------------
// V6: RotateMatrix(ENU, AssimpYUp) == AxisMapper::RotationENUToAssimp
// ---------------------------------------------------------------------------
static void test_rotation_enu_to_assimp()
{
    // Identity rotation
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d old_Rc = AxisMapper::RotationENUToAssimp(R);
    Eigen::Matrix3d new_Rc = CoordinateTransform::RotateMatrix(
        R, CoordinateFrame::ENU, CoordinateFrame::AssimpYUp);

    for (int i = 0; i < 9; ++i) {
        if (std::abs(old_Rc.data()[i]-new_Rc.data()[i]) > EPS) {
            FAIL("mismatch at element " << i << " old="<<old_Rc.data()[i]<<" new="<<new_Rc.data()[i]);
            return;
        }
    }
    PASS();
}

// ---------------------------------------------------------------------------
// V7: BuildRootTransform == AxisMapper::BuildRootTransform
// ---------------------------------------------------------------------------
static void test_build_root_transform()
{
    // Simple ENU rotation (identity with East along X, North along Y, Up along Z)
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T(1000, 2000, 3000);

    Eigen::Matrix4d old_out = AxisMapper::BuildRootTransform(R, T);
    Eigen::Matrix4d new_out = CoordinateTransform::BuildRootTransform(R, T);

    for (int i = 0; i < 16; ++i) {
        if (std::abs(old_out.data()[i]-new_out.data()[i]) > EPS) {
            FAIL("mismatch at element " << i);
            return;
        }
    }
    PASS();
}

// ---------------------------------------------------------------------------
// V8: ConvertNormal == AxisMapper::NormalAssimpToENU
// ---------------------------------------------------------------------------
static void test_normal_assimpYUp_to_enu()
{
    double test_pts[][3] = {
        {1,0,0}, {0,1,0}, {0,0,1}, {0.6,0.8,0},
    };
    for (auto& p : test_pts) {
        double old_ne=0, old_nn=0, old_nu=0;
        AxisMapper::NormalAssimpToENU(p[0], p[1], p[2], old_ne, old_nn, old_nu);

        double new_out[3];
        CoordinateTransform::ConvertNormal(p, CoordinateFrame::AssimpYUp,
                                           new_out, CoordinateFrame::ENU);

        if (std::abs(old_ne-new_out[0]) > EPS ||
            std::abs(old_nn-new_out[1]) > EPS ||
            std::abs(old_nu-new_out[2]) > EPS) {
            FAIL("mismatch");
            return;
        }
    }
    PASS();
}

// ---------------------------------------------------------------------------
// V9: Round-trip identity: AssimpYUp→ENU→AssimpYUp, ZUp→AssimpYUp→ZUp
// ---------------------------------------------------------------------------
static void test_roundtrip()
{
    double test_pts[][3] = {{1,2,3}, {-5,10,-15}, {1e6,2e6,3e6}};
    for (auto& p : test_pts) {
        // AssimpYUp → ENU → AssimpYUp
        {
            double enu[3], back[3];
            CoordinateTransform::Convert(p, CoordinateFrame::AssimpYUp, enu, CoordinateFrame::ENU);
            CoordinateTransform::Convert(enu, CoordinateFrame::ENU, back, CoordinateFrame::AssimpYUp);
            for (int i = 0; i < 3; ++i) {
                if (std::abs(p[i]-back[i]) > EPS) {
                    FAIL("AssimpYUp roundtrip mismatch");
                    return;
                }
            }
        }
        // ZUp → AssimpYUp → ZUp
        {
            double ayu[3], back[3];
            CoordinateTransform::Convert(p, CoordinateFrame::ZUp, ayu, CoordinateFrame::AssimpYUp);
            CoordinateTransform::Convert(ayu, CoordinateFrame::AssimpYUp, back, CoordinateFrame::ZUp);
            for (int i = 0; i < 3; ++i) {
                if (std::abs(p[i]-back[i]) > EPS) {
                    FAIL("ZUp roundtrip mismatch");
                    return;
                }
            }
        }
    }
    PASS();
}

// ---------------------------------------------------------------------------
int main()
{
    std::cerr << "=== CoordinateFrame Verification ===" << std::endl;
    TEST("V1 AssimpYUp→ENU");    test_assimpYUp_to_enu();
    TEST("V2 ENU→AssimpYUp");    test_enu_to_assimpYUp();
    TEST("V3 AssimpYUp→Proj");   test_assimpYUp_to_projected();
    TEST("V4 ZUp→AssimpYUp");    test_zUp_to_assimpYUp();
    TEST("V5 BBox AsYUp→Tiles"); test_bbox_assimpYUp_to_tilesZUp();
    TEST("V6 Rot ENU→Assimp");   test_rotation_enu_to_assimp();
    TEST("V7 BuildRootTransform");test_build_root_transform();
    TEST("V8 Normal AsYUp→ENU"); test_normal_assimpYUp_to_enu();
    TEST("V9 Roundtrip");        test_roundtrip();
    std::cerr << "========================================" << std::endl;
    std::cerr << "PASS=" << g_passed << " FAIL=" << g_failed << std::endl;
    return g_failed > 0 ? 1 : 0;
}
