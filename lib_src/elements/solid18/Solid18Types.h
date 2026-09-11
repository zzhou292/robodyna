// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected native solid startup: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "lib_src/math/Fixed3.h"
#include "lib_src/math/Quaternion.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_SOLID18_HD __host__ __device__
#else
#define TL_SOLID18_HD
#endif

namespace tl::fea::solid18 {
using Vec3 = tl::math::Vec3;
using Matrix3 = tl::math::Matrix3;

enum class Status {
  Success, InvalidInput, UnsupportedProfile, InvalidGeometry, NonfiniteResult
};

struct ResolvedProfile {
  int material_law = 36;
  int native_isolid = 18;
  int engine_jhbe = 17;
  int integration = 2;
  int nptr = 2, npts = 2, nptt = 2;
  int pressure = 2, small_strain = 2, convected_frame = 1;
  // This selected value entry has no thermal, ALE, porosity, reference-shape,
  // initial-stress or automatic closure mode. Source admission is separate.
};

struct ReferenceInput {
  std::uint64_t source_element_id = 0;
  std::uint64_t source_part_id = 0;
  std::uint64_t source_section_id = 0;
  std::uint64_t source_material_id = 0;
  std::uint64_t source_node_id[8]{};
  Vec3 position_m[8]{};  // Original source slots, with supplied SI coordinates.
  double density_kg_m3 = 0;
  ResolvedProfile profile{};
};

struct StartupPoint {
  // BASISF starter order. These are not engine material-history slots.
  double shape[8]{};
  Vec3 derivative_per_m[8]{};  // Native local coordinates and native node slots.
  double jacobian_volume_m3 = 0;
};

struct StartupGeometry {
  Matrix3 frame;  // Native axes in world-space columns.
  Vec3 native_position_m[8]{};
  StartupPoint point[8]{};
  double volume_m3 = 0;
  double native_nodal_volume_m3[8]{};  // Sum(detJ * H), not Gauss volumes.
  Vec3 native_average_derivative_per_m[8]{};
  double characteristic_length_m = 0;  // SDERI3B; not a timestep certificate.
};

struct Mass {
  double source_nodal_mass_kg[8]{};
  double source_nodal_volume_m3[8]{};
  double element_mass_kg = 0;
  // No nodal rotational inertia: these are translational coefficients only.
};

class Reference {
 public:
  TL_SOLID18_HD bool prepared() const noexcept { return prepared_; }
  TL_SOLID18_HD const ReferenceInput& input() const noexcept { return input_; }
  TL_SOLID18_HD const StartupGeometry& geometry() const noexcept { return geometry_; }
  TL_SOLID18_HD const Mass& mass() const noexcept { return mass_; }
  TL_SOLID18_HD unsigned source_slot(unsigned native_slot) const noexcept {
    return native_slot < 8 ? native_to_source_[native_slot] : 8;
  }
 private:
  ReferenceInput input_{};
  StartupGeometry geometry_{};
  Mass mass_{};
  std::uint8_t native_to_source_[8]{};
  bool prepared_ = false;
  friend TL_SOLID18_HD Status InitializeReference(const ReferenceInput&, Reference&) noexcept;
};
}  // namespace tl::fea::solid18
