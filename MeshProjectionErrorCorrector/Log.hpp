// Copyright Johnlyon
//
// MGO::Log — lightweight logging utility for library code
//
// Libraries should use MGO_LOG(level) instead of writing directly to
// std::cout/std::cerr.  The default output stream is std::cerr (conservative
// default for shared libraries).  Applications can redirect via Log::SetOutput()
// and control verbosity via Log::SetLevel().
//
// Usage:
//   MGO_LOG(Info)    << "Processing " << count << " meshes";
//   MGO_LOG(Warning) << "Skipping degenerate mesh " << name;
//   MGO_LOG(Error)   << "Failed to open file: " << path;
//
//   // In application main():
//   MGO::Log::SetOutput(std::cout);
//   MGO::Log::SetLevel(MGO::LogLevel::Info);
//

#pragma once

#include <iostream>
#include <sstream>

namespace MGO {

enum class LogLevel {
    Debug   = 0,
    Info    = 1,
    Warning = 2,
    Error   = 3,
    Silent  = 4
};

struct Log {
    inline static std::ostream* out = &std::cerr;
    inline static LogLevel level    = LogLevel::Warning;

    static void SetOutput(std::ostream& os) { out = &os; }
    static void SetLevel(LogLevel lvl)      { level = lvl; }
};

namespace detail {

class LogLine {
    LogLevel lvl_;
    std::ostringstream ss_;
public:
    explicit LogLine(LogLevel l) : lvl_(l) {}
    ~LogLine() { if (Log::out && !ss_.str().empty()) *Log::out << ss_.str() << std::endl; }
    template<typename T>
    LogLine& operator<<(const T& v) { ss_ << v; return *this; }
    // Support std::endl and other stream manipulators
    LogLine& operator<<(std::ostream& (*pf)(std::ostream&)) {
        ss_ << '\n'; return *this;
    }
};

} // namespace detail
} // namespace MGO

#define MGO_LOG(lvl) \
    if (static_cast<int>(MGO::Log::level) > static_cast<int>(MGO::LogLevel::lvl)) ; \
    else MGO::detail::LogLine(MGO::LogLevel::lvl)
