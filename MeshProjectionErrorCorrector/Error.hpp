// Copyright Johnlyon
//
// MGO::Error — unified error type for all MGO modules
//
// Replaces the previous mix of throw-const-char*, void-with-silent-failure,
// and ad-hoc bool+cerr patterns with a single std::exception-derived type
// carrying an enumerated error code.
//
// Usage:
//   throw MGO::Error(MGO::ErrorCode::FileNotFound, "Cannot open " + path);
//   throw MGO::Error(MGO::ErrorCode::PROJPipelineFailed, proj_errno_string(...));
//
//   try {
//       converter.Convert(opts);
//   } catch (const MGO::Error& e) {
//       std::cerr << "[" << static_cast<int>(e.code()) << "] " << e.what();
//   }
//

#pragma once

#include <stdexcept>
#include <string>

namespace MGO {

enum class ErrorCode {
    Ok = 0,

    // ---- I/O ----
    FileNotFound,
    FileReadError,
    FileWriteError,

    // ---- Mesh data ----
    EmptyScene,
    NoMeshData,
    DegenerateMesh,

    // ---- Projection ----
    InvalidProjectionString,
    PROJPipelineFailed,
    GeoreferencingNotInitialized,

    // ---- Simplification ----
    SimplificationFailed,

    // ---- Control points / fitting ----
    InsufficientControlPoints,
    FitFailed,

    // ---- Internal / logic ----
    InvalidArgument,
    InternalError,
};

class Error : public std::runtime_error {
public:
    Error(ErrorCode code, const std::string& what)
        : std::runtime_error(what), code_(code) {}

    ErrorCode code() const noexcept { return code_; }

private:
    ErrorCode code_;
};

} // namespace MGO
