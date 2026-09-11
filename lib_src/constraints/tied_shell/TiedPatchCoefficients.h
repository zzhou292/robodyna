// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedPatchGeometry.h"

namespace tl::constraints::tied_shell {
struct NodalCoefficients {
  double mass = 0;
  double inertia = 0;
  double translational_stiffness = 0;
  double rotational_stiffness = 0;
};

struct CoefficientInput {
  NodalCoefficients secondary{};
  // Native MINER: entry IN snapshot before this force-stage transfer. The
  // retained field name is historical; this is not immutable time-zero J.
  double initial_master_inertia[4]{};
};

struct CoefficientTransfer {
  // Four ordered increments. Add both repeated slots to a triangle's third node.
  NodalCoefficients master[4]{};
  NodalCoefficients dependent{};
  double source_mass = 0;
  double source_inertia = 0;
  double numerical_mass_delta = 0;  // Native DMAST increment, separate from mass.
};

// Selected explicit I2FOR28_CIN, IRODDL1/WEIGHT1. This stages increments only;
// it does not assemble coefficients, save release history or integrate a node.
TL_TIED_PATCH_HD inline Status TransferCoefficients(
    const Patch& patch, const CoefficientInput& input, CoefficientTransfer& output) noexcept {
  using detail::math::Finite;
  if (!patch.prepared()) return Status::InvalidInput;
  const auto& s = input.secondary;
  const double supplied[] = {s.mass, s.inertia, s.translational_stiffness, s.rotational_stiffness};
  for (double value : supplied) {
    if (!Finite(value) || value < 0) return Status::InvalidInput;
  }
  bool transmit_as_inertia = true;
  for (double value : input.initial_master_inertia) {
    if (!Finite(value) || value < 0) return Status::InvalidInput;
    transmit_as_inertia = transmit_as_inertia && value > 0;
  }
  const auto& p = patch.values();
  const auto& x = p.secondary_offset;
  const auto& c = p.cofactor;
  const double radius_squared = x.x*x.x + x.y*x.y + x.z*x.z;
  const double inx = s.inertia + s.mass*radius_squared;
  const double mrx = c[1] + c[6] + c[5];
  const double mry = c[2] + c[4] + c[6];
  const double mrz = c[3] + c[5] + c[4];
  const double maximum = ::fmax(mrx, ::fmax(mry, mrz));
  const double mr = c[0]*inx*maximum;
  const double fact = transmit_as_inertia ? 0 : 1;
  const double mass = .25*s.mass + mr*fact;
  const double stiffness = .25*s.translational_stiffness + c[0]*maximum*
      (s.rotational_stiffness + s.translational_stiffness*radius_squared);
  const double inertia = inx*.25*(1-fact);
  CoefficientTransfer next;
  for (auto& master : next.master) master = {mass, inertia, stiffness, 0};
  // Exact named native EM20; zero M/J means a dependent DOF, not a small inverse.
  next.dependent = {0, 0, 1e-20, 1e-20};
  next.source_mass = s.mass;
  next.source_inertia = s.inertia;
  next.numerical_mass_delta = 4*mass - s.mass;
  const double results[] = {radius_squared, inx, maximum, mr, mass, stiffness,
                            inertia, next.numerical_mass_delta};
  for (double value : results) {
    if (!Finite(value)) return Status::NonfiniteResult;
  }
  output = next;
  return Status::Success;
}
} // namespace tl::constraints::tied_shell
