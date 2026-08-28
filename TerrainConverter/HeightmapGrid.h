#pragma once

#include "macro.h"
#include <string>
#include <vector>
#include <cstdint>

// HeightmapGrid — regular grid of elevation samples with georeferencing
//
// Coordinate conventions:
//   - heights stored row-major: heights[row * width + col]
//   - row 0 = top (max Northing), row (height-1) = bottom (min Northing)
//   - col 0 = left (min Easting), col (width-1) = right (max Easting)
//   - Geographic bounds in projected CRS (easting/northing, typically Gauss-Kruger or UTM)
//
struct TERRAIN_CONVERTER_API HeightmapGrid
{
    std::vector<float> heights;
    int    width  = 0;
    int    height = 0;

    // Projected bounds
    double minEasting  = 0, maxEasting  = 0;
    double minNorthing = 0, maxNorthing = 0;

    // Sample spacing (abs of pixel-scale terms)
    double dx = 0, dy = 0;

    // GDAL GeoTransform[6]: originX, pixelWidth, rowRot, originY, colRot, pixelHeight.
    // For axis-aligned rasters rowRot=colRot=0 and EastingAt/NorthingAt reduce to
    // the simple linear forms below. For rotated rasters the full 2×2 inverse is
    // used in BilinearSample.
    double GeoTransform[6] = {0,1,0,0,0,-1};

    // No-data
    float  noDataValue = -9999.0f;
    bool   hasNoData   = false;

    // Fill height for noData samples (used by TinSimplifier for vertex output
    // heights).  Set by ProcessTile to 0.0 m (absolute) so all tiles share
    // the same fill elevation and adjacent tile edges align.  The encoding
    // range always includes 0 m, so noData vertices encode linearly without
    // clamping.  Actual noData samples in `heights` keep noDataValue so
    // IsValid() returns false for filtering.
    float  noDataFill  = 0.0f;

    // Projection (from GeoTIFF GeoKeys)
    std::string projectionWKT;     // PROJCS[...] WKT if available
    int         epsg = 0;          // EPSG code if known
    bool        isGeographic = false;  // true if lat/lon (EPSG:4326)

    // Vertical
    double verticalScale = 1.0;    // multiplier to convert stored values to meters

    // Access
    float  HeightAt(int col, int row) const;
    bool   IsValid(int col, int row) const;
    // Geographic → pixel index → value pipeline.
    //
    // The output grid is always axis-aligned (north-up, east-west) regardless
    // of the source GeoTransform orientation.  During ReadElevationGrid each
    // source pixel is re-positioned via:
    //   X_geo = srcT[0] + col*srcT[1] + row*srcT[2]
    //   Y_geo = srcT[3] + col*srcT[4] + row*srcT[5]
    //   outCol = round((X_geo - minEasting) / dx)
    //   outRow = round((maxNorthing - Y_geo) / dy)
    //
    // So the output grid's effective GeoTransform is:
    //   T_out = [minEasting, dx, 0, maxNorthing, 0, -dy]
    //
    // BilinearSample inverts T_out to go from geographic back to pixel
    // coordinates before sampling.

    // Pixel (col, row) → geographic position (output grid, axis-aligned).
    double EastingAt(int col, int row) const { return minEasting + col * dx; }
    double NorthingAt(int col, int row) const { return maxNorthing - row * dy; }
    double EastingAt(int col)        const { return minEasting + col * dx; }
    double NorthingAt(int row)       const { return maxNorthing - row * dy; }

    // Bilinear sample at projected coords (easting, northing); returns false if out of bounds/no-data
    bool   BilinearSample(double easting, double northing, float& outH) const;

    // Min/max of valid samples
    void   ComputeMinMax(float& outMin, float& outMax) const;
};
