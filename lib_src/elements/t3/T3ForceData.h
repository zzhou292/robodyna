// SPDX-License-Identifier: AGPL-3.0-or-later
// Native T3 field conventions, OpenRadioss (C) 2026 Siemens; see LICENSE.md.
#pragma once
#include "T3HistoryData.h"
#include "T3KinematicsData.h"
namespace tl::fea::t3 {
struct ForceDiagnostics {
  double effective_thickness=0,native_sound_speed=0; // m, m/s.
  double membrane_viscosity=0,shear_factor=0,transverse_shear_modulus=0; // DM, SHF, GS(Pa).
  double translational_stiffness=0,rotational_stiffness=0; // Native STI N/m, STIR N*m.
  double unscaled_element_dt=0; // C3DT3 index7 diagnostic, not stability admission.
  double internal_work_increment[2]{}; // J, difference of stored EINT values.
};
struct ForceTrial {
  History proposed_history;
  Kinematics kinematics; // Unchanged 38 R2 observables before strain/length mutations.
  Vec3 internal_force[3]{},internal_couple[3]{}; // Positive native world N, N*m.
  ForceDiagnostics diagnostics;
};
static_assert(sizeof(ForceTrial)<2048,"Bounded prescribed three-node force packet");
} // namespace tl::fea::t3
