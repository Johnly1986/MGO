// Copyright Johnlyon
//
// SimplifyOptions — unified mesh simplification parameters
//
// Used by TilesConverter, TerrainConverter, OSGBConverter, and MGOConsole.
// Each module may set its own defaults; CLI flags override.
//

#pragma once

struct SimplifyOptions
{
    float error        = 0.0f;   // 0 = disabled, >0 = enabled
    float normalWeight = 0.1f;   // Normal attribute weight
    float threshold    = 0.1f;   // 0 = error-driven, >0 = ratio of original indices
    bool  lockBorder   = false;   // Lock border vertices
    bool  localError   = false;  // Use absolute/local error metric

    bool enabled() const { return error > 0.0f; }
};