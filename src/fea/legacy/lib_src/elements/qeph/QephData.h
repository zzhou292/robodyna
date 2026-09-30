// SPDX-License-Identifier: AGPL-3.0-or-later
// QEPH field conventions adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Pin/source map: lib_utest/qualification/qeph/source-manifest.json.
#pragma once
#include "lib_src/elements/ShellReferencePlacement.h"

#include "lib_src/math/Fixed3.h"
#include <cstdint>

namespace tl::fea::qeph {
using Vec3 = tl::math::Vec3;
using Matrix3 = tl::math::Matrix3;

enum class Status {
  kSuccess, kInvalidInput, kUnsupportedGeometry, kNonfiniteResult, kInvalidReference,
};

// One cyclic native Q4, typed reference placement, SI units.
// This record does not identify a nodal owner, epoch or clock.
struct ReferenceInput {
  Vec3 position[4]{};
  std::uint32_t node_ids[4]{0,1,2,3};
  double density=7890;          // kg/m^3
  double young_modulus=200e9;   // Pa; retained immutable LAW1 input.
  double poisson_ratio=.3;     // First branch: 0<=nu<.5.
  double thickness=.001648;    // m
  ShellReferencePlacement placement=ShellReferencePlacement::Centered;
  // Immutable numerical metric for the native mixed translation/rotation
  // projection. Physical inputs/outputs stay SI. Default1 preserves legacy.
  double projection_working_length_m=1;
};

struct ReferenceData {
  ReferenceInput input;
  Matrix3 frame;                // Row-major; world basis vectors are columns.
  double area=0;                // Native mean-plane projected area, m^2.
  double derivative_x[4]{};     // CDERII unnormalized coefficients, m.
  double derivative_y[4]{};
  Vec3 local_position[4]{};     // Projected node0-relative x/y; reported z=0.
  double nodal_mass[4]{};       // rho*t*A/4, kg.
  double physical_inertia[4]{}; // Native thickness/offset partition, kg*m^2.
  double added_inertia[4]{};    // Native m*(A/12) partition, kg*m^2.
  double isotropic_inertia[4]{};// Exact native total expression, not recombined.
  bool prepared=false;
};
}  // namespace tl::fea::qeph
