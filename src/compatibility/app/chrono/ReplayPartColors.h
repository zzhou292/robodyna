#pragma once
#include "ReplayGeometryLimits.h"
#include "chrono/assets/ChColor.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace crash::visual {
inline constexpr const char* ReplayPartPaletteName = "robo-dyna.original-part-id.v1";
inline constexpr std::uint64_t ReplayPartPaletteSeed = 1;
inline constexpr std::size_t ReplayPartColorTriangleLimit = ReplayGeometryLimits::Vehicle().triangles;
struct ReplayPartLegendEntry {
    std::uint64_t part_id = 0;
    chrono::ChColor color;
};
// Versioned integer mixing followed by an HSV color construction. Depends only
// on the full original PID and explicit seed, never a frame, part order or selected subset.
// Categorical colors carry no stress/material meaning. Arbitrary 64-bit IDs
// cannot all have unique display colors; the mapping makes no such promise.
chrono::ChColor ReplayPartColor(std::uint64_t part_id,
    std::uint64_t seed = ReplayPartPaletteSeed) noexcept;

class ReplayPartColors {
  public:
    // Requires complete positive original PIDs and at most the explicit 1 Mi
    // triangle bound; failure preserves colors and the prior legend. Allocates
    // only during initialization. Reader/scene geometry admission is separate.
    bool Initialize(const std::vector<std::uint64_t>& triangle_parts,
                    std::vector<chrono::ChColor>& colors, std::uint64_t seed = ReplayPartPaletteSeed);
    const std::vector<ReplayPartLegendEntry>& legend() const noexcept { return legend_; }
  private:
    std::vector<ReplayPartLegendEntry> legend_;
};
} // namespace crash::visual
