// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid18/Solid18Types.h"

namespace tl::fea::solid18::total_strain {
// Exact selected native allocation. Coordinates/derivatives use SI units.
struct ReferenceCoefficients {
  double center_jacobian_inverse_per_m[9]{}; // Native G_JAC_I slots1..9.
  double center_determinant_m3 = 0;         // Slot10, distinct from Gauss volumes.
  double point_pij_per_m[8][72]{};          // Native IP=r+2*s+4*t.
  Vec3 source_relative_position_m[7]{};    // SGSAVINI before H8 reordering.
};

struct ReferenceScratch;
class Reference {
 public:
  TL_SOLID18_HD bool prepared() const noexcept { return prepared_; }
  TL_SOLID18_HD const ReferenceInput& input() const noexcept { return input_; }
  TL_SOLID18_HD const StartupGeometry& geometry() const noexcept { return geometry_; }
  TL_SOLID18_HD const Mass& mass() const noexcept { return mass_; }
  TL_SOLID18_HD const ReferenceCoefficients& coefficients() const noexcept { return coefficients_; }
  TL_SOLID18_HD unsigned source_slot(unsigned native_slot) const noexcept {
    return native_slot < 8 ? native_to_source_[native_slot] : 8;
  }
 private:
  ReferenceInput input_{};
  StartupGeometry geometry_{};
  Mass mass_{};
  ReferenceCoefficients coefficients_{};
  std::uint8_t native_to_source_[8]{};
  bool prepared_ = false;
  friend TL_SOLID18_HD Status InitializeReference90Scratch(const ReferenceInput&, ReferenceScratch&) noexcept;
};

TL_SOLID18_HD inline ResolvedProfile Law90Profile() noexcept {
  ResolvedProfile profile;
  profile.material_law = 90;
  profile.pressure = 0;
  profile.small_strain = 10;
  return profile;
}
}  // namespace tl::fea::solid18::total_strain
