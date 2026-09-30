// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected OpenRadioss TYPE2/Spotflag28 kinematic branch.
// Native arithmetic: Copyright (C) 2026 Siemens.
#pragma once
#include "lib_src/math/Fixed3Operations.h"

#if defined(__CUDACC__)
#define TL_TIED_PATCH_HD __host__ __device__
#else
#define TL_TIED_PATCH_HD
#endif

namespace tl::constraints::tied_shell {
using Vec3 = tl::math::Vec3;
enum class Status { Success, InvalidInput, SingularPatch, NonfiniteResult };

struct PatchInput {
  // Native four ordered slots. A triangular master repeats its third node in
  // slot four; do not replace these slots with three equal weights.
  Vec3 master_position[4]{};
  Vec3 secondary_position{};
};

struct PatchValues {
  Vec3 center{};
  Vec3 master_offset[4]{};
  Vec3 secondary_offset{};
  // Native DPARA: inverse determinant, B1, B2, B3, C1, C2, C3.
  double cofactor[7]{};
};

class Patch {
 public:
  TL_TIED_PATCH_HD bool prepared() const noexcept { return prepared_; }
  TL_TIED_PATCH_HD const PatchValues& values() const noexcept { return values_; }
 private:
  PatchValues values_{};
  bool prepared_ = false;
  friend TL_TIED_PATCH_HD Status PreparePatch(const PatchInput&, Patch&) noexcept;
};

struct SecondaryLoad {
  Vec3 force{};
  Vec3 couple{};
};
struct MasterLoads {
  Vec3 force[4]{};
};
struct MasterMotion {
  Vec3 velocity[4]{};
  Vec3 acceleration[4]{};
};
struct SecondaryMotion {
  Vec3 velocity{};
  Vec3 angular_velocity{};
  Vec3 acceleration{};
  Vec3 angular_acceleration{};
};
} // namespace tl::constraints::tied_shell
