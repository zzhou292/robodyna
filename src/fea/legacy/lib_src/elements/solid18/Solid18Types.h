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
  // Engine IP = r + 2*s + 4*t, with r/s/t in {0,1}.
  // S8EJACIP3's scaled covariant rows; each row contains local x/y/z.
  Matrix3 scaled_jacobian_m;
  double initial_volume_m3 = 0;  // Native S8EDERI3 LBUF%VOL and VOL0DP.
};

struct StartupGeometry {
  Matrix3 frame;  // Native axes in world-space columns.
  Vec3 native_position_m[8]{};
  Matrix3 center_scaled_jacobian_m;
  Vec3 higher_mode_m[4]{};  // Native HX/HY/HZ modes, not shape weights.
  StartupPoint point[8]{};
  double center_volume_m3 = 0;  // S8ZDERIC3 GBUF%VOL / VOLU.
  double integrated_volume_m3 = 0;  // Diagnostic sum of the eight point volumes.
  double inverse_center_face_scale_per_m2 = 0;
  double characteristic_length_m = 0;  // Selected S8EDERI3; no dt certificate.
};

struct Mass {
  double source_nodal_mass_kg[8]{};
  double element_mass_kg = 0;
  double initial_global_density_kg_m3 = 0;  // SVALUE0 sum in native visitation order.
  // SMASS3 uses this global density and center volume. No rotational inertia.
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
