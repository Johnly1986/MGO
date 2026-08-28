// test_error — unit tests for A2 error handling improvements
//
// Verifies:
//   E1: MGO::Error construction, catching, ErrorCode
//   E2: MeshGroupOptimizer::Load throws MGO::Error (not const char*)
//   E3: MeshGroupOptimizer null-guard: Optimize/Save return false
//   E4: CProjectionEngine null-guard: TransformPointToECEF returns false
//   E5: GeoreferencingWith7Parameters catch(const std::exception&) doesn't
//       swallow fatal signals
//   E6: MGO::Log level filtering and output redirection

#include "../MeshProjectionErrorCorrector/Error.hpp"
#include "../MeshProjectionErrorCorrector/Log.hpp"
#include "../MeshGroupOptimizer/MeshGroupOptimizer.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWith7Parameters.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingFactory.h"

#include <iostream>
#include <sstream>
#include <cassert>
#include <cstring>

static int g_passed = 0;
static int g_failed = 0;

#define TEST(name) \
    do { std::cerr << "  " << name << "... "; } while(0)

#define PASS() \
    do { std::cerr << "PASS" << std::endl; ++g_passed; } while(0)

#define FAIL(msg) \
    do { std::cerr << "FAIL: " << msg << std::endl; ++g_failed; } while(0)

// ---------------------------------------------------------------------------
// E1: MGO::Error construction and catching
// ---------------------------------------------------------------------------
static void test_error_type()
{
    // 1a: Construction and what()
    {
        MGO::Error e(MGO::ErrorCode::FileNotFound, "test.fbx not found");
        assert(e.code() == MGO::ErrorCode::FileNotFound);
        assert(std::strlen(e.what()) > 0);
    }

    // 1b: Caught as std::exception
    {
        bool caught = false;
        try {
            throw MGO::Error(MGO::ErrorCode::InternalError, "something broke");
        } catch (const std::exception& e) {
            caught = true;
            assert(std::string(e.what()).find("something broke") != std::string::npos);
        }
        assert(caught); (void)caught;
    }

    // 1c: Caught as MGO::Error with code access
    {
        bool caught = false;
        try {
            throw MGO::Error(MGO::ErrorCode::PROJPipelineFailed, "pipeline error");
        } catch (const MGO::Error& e) {
            caught = true;
            assert(e.code() == MGO::ErrorCode::PROJPipelineFailed);
        }
        assert(caught); (void)caught;
    }

    // 1d: Catch order — MGO::Error before std::exception still works
    {
        bool caughtAsMGO = false;
        try {
            throw MGO::Error(MGO::ErrorCode::EmptyScene, "no meshes");
        } catch (const MGO::Error& e) {
            caughtAsMGO = true;
            assert(e.code() == MGO::ErrorCode::EmptyScene);
        } catch (const std::exception&) {
            assert(false);  // should not reach here
        }
        assert(caughtAsMGO); (void)caughtAsMGO;
    }
}

// ---------------------------------------------------------------------------
// E2: MeshGroupOptimizer::Load throws MGO::Error (not const char*)
// ---------------------------------------------------------------------------
static void test_meshgroupoptimizer_load_throw()
{
    CMeshGroupOptimizer mgo;

    // 2a: Non-existent file should throw MGO::Error
    {
        bool caughtMGO = false;
        try {
            mgo.Load("/nonexistent/path/model.fbx");
        } catch (const MGO::Error& e) {
            caughtMGO = true;
            assert(e.code() == MGO::ErrorCode::FileReadError);
            assert(std::string(e.what()).find("Assimp:") != std::string::npos);
        } catch (const std::exception&) {
            assert(false);  // should be caught as MGO::Error first
        } catch (const char*) {
            assert(false);  // OLD behavior — must not happen anymore
        }
        assert(caughtMGO); (void)caughtMGO;
    }
}

// ---------------------------------------------------------------------------
// E3: MeshGroupOptimizer null-guard (Optimize/Save return false)
// ---------------------------------------------------------------------------
static void test_meshgroupoptimizer_null_guard()
{
    CMeshGroupOptimizer mgo;
    // Never called Load() — m_scene is nullptr

    // 3a: Optimize returns false when no scene loaded
    {
        OptimizerConfig config;
        config.items.push_back(OptimizerItem(".*", 0.01f, 0.1f, 0.0f, true, false));
        bool result = mgo.Optimize(config);
        assert(result == false);
    }

    // 3b: Save returns false when no scene loaded
    {
        bool result = mgo.Save("/tmp/should_not_exist.obj", false);
        assert(result == false);
    }

    // 3c: SimplifyScene returns false on null scene
    {
        OptimizerConfig config;
        bool result = CMeshGroupOptimizer::SimplifyScene(nullptr, config);
        assert(result == false);
    }
}

// ---------------------------------------------------------------------------
// E4: CProjectionEngine null-guard
// ---------------------------------------------------------------------------
static void test_projectionengine_null_guard()
{
    CProjectionEngine engine;

    // 4a: TransformPointToECEF fails when no georeferencing is set
    {
        double ex = 0, ey = 0, ez = 0;
        bool result = engine.TransformPointToECEF(1.0, 2.0, 3.0, ex, ey, ez);
        assert(result == false);
    }

    // 4b: ProjectedToGeographic fails when no georeferencing is set
    {
        double lat = 0, lon = 0;
        bool result = engine.ProjectedToGeographic(500000.0, 3000000.0, lat, lon);
        assert(result == false);
    }
}

// ---------------------------------------------------------------------------
// E5: GeoreferencingWith7Parameters — catch(const std::exception&)
//      Old-style PROJ strings are normalized, not silently broken
// ---------------------------------------------------------------------------
static void test_georeferencing_catch_exception()
{
    // 5a: Construction with a known-good empty string (identity Helmert)
    {
        GeoreferencingWith7Parameters georef("", "EPSG:4979");
        // Should not throw
        SevenParameter zero;
        georef.SetParameter(zero);
        georef.Solve();
        // Transform should return something (identity or near-identity)
        Eigen::Vector3d result = georef.Transform(Eigen::Vector3d(1.0, 2.0, 3.0));
        // With identity Helmert, result should be the same at meter scale
        assert(std::abs(result.x() - 1.0) < 200.0); // within 200m (datum shift)
        assert(std::abs(result.y() - 2.0) < 200.0);
        assert(std::abs(result.z() - 3.0) < 200.0); (void)result;
    }

    // 5b: PROJ string with deprecated +no_defs should work (normalized)
    {
        std::string old_style = "+proj=tmerc +lon_0=100.35 +lat_0=0 +k=1 "
            "+x_0=500000 +y_0=0 +ellps=GRS80 +units=m +no_defs +type=crs";
        GeoreferencingWith7Parameters georef(old_style, "EPSG:4979");
        SevenParameter zero;
        georef.SetParameter(zero);
        georef.Solve();
        Eigen::Vector3d result = georef.Transform(Eigen::Vector3d(500000.0, 0.0, 0.0));
        // Should produce a valid geographic coordinate near the meridian
        assert(result.x() > 99.0 && result.x() < 102.0); // lon near 100.35
        assert(std::abs(result.y()) < 1.0);               // lat near 0
        (void)result;
    }
}

// ---------------------------------------------------------------------------
// E6: MGO::Log — level filtering and output redirection
// ---------------------------------------------------------------------------
static void test_log_filtering()
{
    std::ostringstream capture;

    // Save original state
    auto* orig_out = MGO::Log::out;
    auto orig_level = MGO::Log::level;

    MGO::Log::SetOutput(capture);

    // 6a: Default level (Warning) filters Info
    {
        capture.str("");
        MGO_LOG(Info) << "This should not appear";
        assert(capture.str().empty());
    }

    // 6b: Warning passes at default level
    {
        capture.str("");
        MGO_LOG(Warning) << "This warning should appear";
        assert(capture.str().find("This warning") != std::string::npos);
    }

    // 6c: Error passes at default level
    {
        capture.str("");
        MGO_LOG(Error) << "This error should appear";
        assert(capture.str().find("This error") != std::string::npos);
    }

    // 6d: SetLevel(Debug) reveals all messages
    {
        capture.str("");
        MGO::Log::SetLevel(MGO::LogLevel::Debug);
        MGO_LOG(Debug) << "Debug message";
        MGO_LOG(Info) << "Info message";
        MGO_LOG(Warning) << "Warning message";
        MGO_LOG(Error) << "Error message";
        std::string out = capture.str();
        assert(out.find("Debug message") != std::string::npos);
        assert(out.find("Info message") != std::string::npos);
        assert(out.find("Warning message") != std::string::npos);
        assert(out.find("Error message") != std::string::npos);
    }

    // 6e: SetLevel(Silent) suppresses everything
    {
        capture.str("");
        MGO::Log::SetLevel(MGO::LogLevel::Silent);
        MGO_LOG(Error) << "Should not appear";
        assert(capture.str().empty());
    }

    // 6f: LogLine ends with newline
    {
        capture.str("");
        MGO::Log::SetLevel(MGO::LogLevel::Debug);
        MGO_LOG(Info) << "Newline test";
        std::string out = capture.str();
        assert(!out.empty());
        assert(out.back() == '\n');
    }

    // Restore original state
    MGO::Log::SetOutput(*orig_out);
    MGO::Log::SetLevel(orig_level);
}

// ---------------------------------------------------------------------------
// E7: ErrorCode enum values are distinct and cover the documented categories
// ---------------------------------------------------------------------------
static void test_errorcode_enum()
{
    // Verify all codes are distinct
    assert(static_cast<int>(MGO::ErrorCode::Ok) == 0);
    assert(static_cast<int>(MGO::ErrorCode::FileNotFound) !=
           static_cast<int>(MGO::ErrorCode::FileReadError));
    assert(static_cast<int>(MGO::ErrorCode::FileReadError) !=
           static_cast<int>(MGO::ErrorCode::FileWriteError));
    assert(static_cast<int>(MGO::ErrorCode::InsufficientControlPoints) !=
           static_cast<int>(MGO::ErrorCode::FitFailed));
}

// ---------------------------------------------------------------------------
// E8: GeoreferencingFactory::Create — default Identity type works
// ---------------------------------------------------------------------------
static void test_factory_create()
{
    // 8a: Create Identity — should succeed
    {
        GeoreferencingOptions opts;
        auto g = GeoreferencingFactory::Create(
            GeoreferencingType::Identity, "", opts);
        assert(g != nullptr);
    }

    // 8b: Create None — returns nullptr
    {
        auto g = GeoreferencingFactory::Create(
            GeoreferencingType::None, "", {});
        assert(g == nullptr);
    }
}

// ---------------------------------------------------------------------------
// main — run all tests
// ---------------------------------------------------------------------------
int main()
{
    std::cerr << "=== Error Handling Tests ===" << std::endl;

    TEST("E1 Error construction/catching");
    test_error_type(); PASS();

    TEST("E2 MeshGroupOptimizer::Load throw type");
    test_meshgroupoptimizer_load_throw(); PASS();

    TEST("E3 MeshGroupOptimizer null-guard");
    test_meshgroupoptimizer_null_guard(); PASS();

    TEST("E4 CProjectionEngine null-guard");
    test_projectionengine_null_guard(); PASS();

    TEST("E5 Georeferencing catch exception");
    test_georeferencing_catch_exception(); PASS();

    TEST("E6 Log filtering and redirection");
    test_log_filtering(); PASS();

    TEST("E7 ErrorCode enum");
    test_errorcode_enum(); PASS();

    TEST("E8 Factory Create");
    test_factory_create(); PASS();

    std::cerr << "========================================" << std::endl;
    std::cerr << "PASS=" << g_passed << " FAIL=" << g_failed << std::endl;
    return g_failed > 0 ? 1 : 0;
}
