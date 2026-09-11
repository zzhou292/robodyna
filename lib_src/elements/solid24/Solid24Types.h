// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid_common/BrickFrame.h"
#include <cstdint>

namespace tl::fea::solid24 {
using Vec3 = tl::math::Vec3;
using Matrix3 = tl::math::Matrix3;
enum class Status { Success, InvalidInput, UnsupportedProfile, InvalidGeometry, NonfiniteResult };

// The SINIT3 startup call, after INITIA's local JCVT override. Runtime frame,
// pressure, material and strain-formulation admission are separate contracts.
struct StartupProfile {
  int engine_jhbe = 24, integration_points = 1, startup_frame = 1;
  int rotational_inertia = 0, ale = 0, reference_shape = 0;
};
struct ReferenceInput {
  std::uint64_t source_element_id = 0, source_part_id = 0;
  std::uint64_t source_section_id = 0, source_material_id = 0;
  std::uint64_t source_node_id[8]{};
  Vec3 position_m[8]{};
  double density_kg_m3 = 0;  // Caller-prepared virgin GBUF density, not a material resolver.
  StartupProfile profile{};
};
struct StartupGeometry {
  Matrix3 frame;  // SRCOOR3 HEPH cyclic axes, in world-space columns.
  Vec3 local_position_m[8]{};
  double volume_m3 = 0, characteristic_length_m = 0;
};
struct Mass {
  double source_slot_mass_kg[8]{};
  double element_mass_kg = 0;
  // This HEPH startup accepts eight distinct nodes. Six-node source cells use
  // the earlier S6ZINIT3 dispatcher and require a separate wedge participant.
  static constexpr double isotropic_inertia_kg_m2() noexcept { return 0; }
};
class Reference {
 public:
  TL_BRICK_HD bool prepared() const noexcept { return prepared_; }
  TL_BRICK_HD const ReferenceInput& input() const noexcept { return input_; }
  TL_BRICK_HD const StartupGeometry& geometry() const noexcept { return geometry_; }
  TL_BRICK_HD const Mass& mass() const noexcept { return mass_; }
  TL_BRICK_HD unsigned source_slot(unsigned n) const noexcept { return n < 8 ? native_to_source_[n] : 8; }
  TL_BRICK_HD unsigned unique_node_count() const noexcept { return unique_node_count_; }
 private:
  ReferenceInput input_{};
  StartupGeometry geometry_{};
  Mass mass_{};
  std::uint8_t native_to_source_[8]{}, unique_node_count_ = 0;
  bool prepared_ = false;
  friend TL_BRICK_HD Status InitializeReference(const ReferenceInput&, Reference&) noexcept;
};
}  // namespace tl::fea::solid24
