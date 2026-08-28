#include "HeightmapGrid.h"
#include <algorithm>
#include <cmath>
#include <limits>

float HeightmapGrid::HeightAt(int col, int row) const
{
    return heights[static_cast<size_t>(row) * width + col];
}

bool HeightmapGrid::IsValid(int col, int row) const
{
    if (col < 0 || col >= width || row < 0 || row >= height) return false;
    float h = HeightAt(col, row);
    if (hasNoData && std::abs(h - noDataValue) < 1e-6f) return false;
    return true;
}

bool HeightmapGrid::BilinearSample(double easting, double northing, float& outH) const
{
    if (width <= 1 || height <= 1) return false;

    // Output grid effective GeoTransform: [minE, dx, 0, maxN, 0, -dy]
    //   X_geo = minEasting + col * dx
    //   Y_geo = maxNorthing - row * dy
    //
    // Inverse (geographic → fractional pixel):
    //   col = (X_geo - minEasting) / dx
    //   row = (maxNorthing - Y_geo) / dy
    //
    // This is the exact 2×2 inverse of the output grid's axis-aligned
    // transform. For non-rotated sources the result is identical to
    // inverting the source GeoTransform; for rotated sources the output
    // grid is axis-aligned by construction and the source GeoTransform
    // was only used during the write phase to position each pixel.

    double fc = (easting - minEasting) / dx;
    double fr = (maxNorthing - northing) / dy;

    // Clamp sub-ULP overshoot at boundaries (e.g. fc=63.00000000001 at W=64
    // due to binary floating-point division), but reject truly out-of-bounds
    // queries that would read past the array.
    if (fc < -1e-9 || fc > width - 1 + 1e-9 ||
        fr < -1e-9 || fr > height - 1 + 1e-9) return false;

    fc = std::max(0.0, std::min(fc, static_cast<double>(width - 1)));
    fr = std::max(0.0, std::min(fr, static_cast<double>(height - 1)));

    int c0 = static_cast<int>(std::floor(fc));
    int r0 = static_cast<int>(std::floor(fr));
    int c1 = std::min(c0 + 1, width - 1);
    int r1 = std::min(r0 + 1, height - 1);

    double tc = fc - c0;
    double tr = fr - r0;

    float h00 = HeightAt(c0, r0);
    float h10 = HeightAt(c1, r0);
    float h01 = HeightAt(c0, r1);
    float h11 = HeightAt(c1, r1);

    if (hasNoData &&
        (std::abs(h00 - noDataValue) < 1e-6f ||
         std::abs(h10 - noDataValue) < 1e-6f ||
         std::abs(h01 - noDataValue) < 1e-6f ||
         std::abs(h11 - noDataValue) < 1e-6f))
    {
        return false;
    }

    double h0 = h00 * (1.0 - tc) + h10 * tc;
    double h1 = h01 * (1.0 - tc) + h11 * tc;
    outH = static_cast<float>(h0 * (1.0 - tr) + h1 * tr);
    return true;
}

void HeightmapGrid::ComputeMinMax(float& outMin, float& outMax) const
{
    outMin =  std::numeric_limits<float>::max();
    outMax = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < heights.size(); ++i)
    {
        float h = heights[i];
        if (hasNoData && std::abs(h - noDataValue) < 1e-6f) continue;
        if (h < outMin) outMin = h;
        if (h > outMax) outMax = h;
    }
    if (outMin > outMax) { outMin = 0.0f; outMax = 0.0f; }
}
