// SPDX-License-Identifier: AGPL-3.0-or-later
// Native QEPH field conventions, OpenRadioss (C) 2026 Siemens; see LICENSE.md.
#pragma once
#include "QephHistoryData.h"
#include "QephKinematicsData.h"

namespace tl::fea::qeph {
struct ForceDiagnostics {
  double effective_thickness=0,native_sound_speed=0;
  double membrane_viscosity=0,stabilization_viscosity=0; // Native DM/DN=.015.
  // Native CNDT3 before FACN scatter, not a stability certificate/admission.
  double translational_stiffness=0; // N/m.
  double rotational_stiffness=0;    // N*m.
  double unscaled_element_dt=0;    // DTEL, s; DTFAC1(3)=1.
  double internal_work_increment[2]{}; // J, distinct from accumulated EINT.
  double hourglass_viscous_work_increment=0; // J.
};
struct ForceTrial {
  History proposed_history;
  Kinematics kinematics; // Same pre-CNDT3 observables as EvaluatePrescribed.
  Vec3 internal_force[4]{};  // Positive native world internal force, N.
  Vec3 internal_couple[4]{}; // Positive native world couple, N*m.
  ForceDiagnostics diagnostics;
};
} // namespace tl::fea::qeph
