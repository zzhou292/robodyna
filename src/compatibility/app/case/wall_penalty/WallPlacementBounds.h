#pragma once
#include "lib_src/collision/PlanarWallBox.h"
#include "lib_src/collision/Q4ContactBounds.h"
#include <array>
namespace crash::cases::wall_penalty {
// Value-only outward bounds. X remains at the source bounds here; the owning
// geometry adapter projects both endpoints to the placed wall for coverage.
// Failure preserves output. No wall/source coordinate is changed.
bool ExpandProjectedMotion(const std::array<tlfea::contact::Vec3,2>& reference,double margin,
                           tlfea::contact::PlanarWallBox* output) noexcept;
bool EncloseLeadingGap(double actual_wall_x,double leading_reference_x,
                       tlfea::contact::Q4IntegralInterval* output) noexcept;
} // namespace crash::cases::wall_penalty
