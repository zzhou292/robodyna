// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid_common/BrickStartup.h"
#include <cstdint>

namespace tl::fea::solid6z {
using Vec3 = tl::math::Vec3;
using Matrix3 = tl::math::Matrix3;
enum class Status { Success, InvalidInput, UnsupportedProfile, InvalidGeometry, NonfiniteResult };

// Explicit selected S6ZINIT3 geometry/mass profile. Material resolution and
// the eventual force policy are separate, authenticated caller obligations.
struct StartupProfile {
  int engine_jhbe = 24, integration_points = 1, strain_formulation = 10;
  int mass_distribution = 0, orthotropic_frame = 0, thermal = 0;
  int ale = 0, reference_shape = 0;
};
struct ReferenceInput {
  std::uint64_t source_element_id = 0, source_part_id = 0;
  std::uint64_t source_section_id = 0, source_material_id = 0;
  std::uint64_t source_node_id[6]{};
  Vec3 position_m[6]{};  // Two paired triangle faces; no implicit arity conversion.
  double density_kg_m3 = 0;  // Virgin GBUF density supplied by the material caller.
  StartupProfile profile{};
};
struct StartupGeometry {
  Matrix3 frame;  // Native cyclic frame columns, in world coordinates.
  Vec3 local_position_m[6]{};
  double inverse_reference_jacobian[9]{};  // Native JAC_I(1:9), explicit order.
  double reference_volume_m3 = 0;  // JAC_I(10), world reference geometry.
  double volume_m3 = 0, axial_volume_gradient_m3 = 0;
  double characteristic_length_m = 0;
};
struct Mass {
  double source_slot_mass_kg[6]{};
  double element_mass_kg = 0;
  static constexpr double isotropic_inertia_kg_m2() noexcept { return 0; }
};
class Reference {
 public:
  TL_BRICK_HD bool prepared() const noexcept { return prepared_; }
  TL_BRICK_HD const ReferenceInput& input() const noexcept { return input_; }
  TL_BRICK_HD const StartupGeometry& geometry() const noexcept { return geometry_; }
  TL_BRICK_HD const Mass& mass() const noexcept { return mass_; }
  TL_BRICK_HD unsigned source_slot(unsigned n) const noexcept {
    return n < 6 ? native_to_source_[n] : 6;
  }
 private:
  ReferenceInput input_{};
  StartupGeometry geometry_{};
  Mass mass_{};
  std::uint8_t native_to_source_[6]{};
  bool prepared_ = false;
  friend TL_BRICK_HD Status InitializeReference(const ReferenceInput&, Reference&) noexcept;
};
}  // namespace tl::fea::solid6z
