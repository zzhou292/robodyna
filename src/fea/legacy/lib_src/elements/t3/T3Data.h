// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from pinned OpenRadioss, Copyright (C) 2026 Siemens.
// See LICENSE.md and lib_utest/qualification/t3/source-manifest.json.
#pragma once
#include "lib_src/elements/ShellReferencePlacement.h"
#include "lib_src/math/Fixed3.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_T3_HD __host__ __device__
#else
#define TL_T3_HD
#endif

namespace tl::fea::t3 {
using Vec3=tl::math::Vec3;
using Matrix3=tl::math::Matrix3;
enum class Status { kSuccess,kInvalidInput,kUnsupportedGeometry,kNonfiniteResult,kInvalidReference };
struct ReferenceInput {
  Vec3 position[3]{};
  std::uint64_t node_ids[3]{0,1,2};
  double density=7890,thickness=.001648,young_modulus=200e9,poisson_ratio=.3;
  ShellReferencePlacement placement=ShellReferencePlacement::Centered;
};
// Immutable producer result; no owner/clock or material state. The selected
// native T3 angle masses and A/4.5 inertia must not become Q4 equal quarters.
struct ReferenceData {
  ReferenceInput input;
  Matrix3 frame; // Row-major, world basis vectors in columns.
  double area=0;
  Vec3 local_position[3]{};
  double angle_cosine[3]{},angle_weight[3]{},nodal_mass[3]{};
  double physical_inertia[3]{},added_inertia[3]{},isotropic_inertia[3]{};
  double element_mass=0,element_isotropic_inertia=0;
  double element_physical_inertia=0,element_added_inertia=0;
  double characteristic_length=0,startup_derivative[3]{};
  bool prepared=false;
};
static_assert(sizeof(ReferenceData)<1024,"Bounded three-node reference");
} // namespace tl::fea::t3
