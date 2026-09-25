// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_src/math/Fixed3.h"
namespace tlfea::contact::radioss_type25 {
using Vector = tl::math::Vec3;
struct FrictionControls {
  int model = -1, formulation = -1, orthotropic = -1, converged = -1;
  int thermal = -1, part_coefficients = -1;
  double alpha = 0;
};
template<class Units> struct FrictionCoefficients {
  double base = 0;
  double c[6]{}; // Native Darmstad ordering. Units differ by coefficient.
};
template<class Units> struct FrictionInput {
  NormalInput<Units> normal;
  Vector normal_axis{}, relative_velocity{};
  Vector main_vertices[4]{};
  double dt12 = 0; // Native DT12, independent of DT1=normal.dt.
};
template<class Units> struct FrictionHistory {
  NormalHistory<Units> normal;
  Vector previous_force{}, staged_force{}; // SECND_FR(4:6), SECND_FR(1:3).
};
template<class Units> struct FrictionResult {
  NormalResult<Units> normal;
  FrictionHistory<Units> history;
  Vector tangent_predictor{}, tangent_force{}, native_resultant{};
  double coefficient = 0, limiter = 0, contact_area = 0, pressure = 0;
  double friction_work = 0;
  bool contact_active = false; // False: force/geometry scratch is not a native observation.
};
using NativeFrictionInput = FrictionInput<NativeUnitsTag>;
using SiFrictionInput = FrictionInput<SiUnitsTag>;
using NativeFrictionHistory = FrictionHistory<NativeUnitsTag>;
using SiFrictionHistory = FrictionHistory<SiUnitsTag>;
using NativeFrictionCoefficients = FrictionCoefficients<NativeUnitsTag>;
using SiFrictionCoefficients = FrictionCoefficients<SiUnitsTag>;
using NativeFrictionResult = FrictionResult<NativeUnitsTag>;
using SiFrictionResult = FrictionResult<SiUnitsTag>;
// Native row-state phase helpers are in working units. They preserve the full
// PENE_OLD payload, including slots4/5 which the response packet does not update.
struct NativeContactRow {
  NativeFrictionHistory history;
  double penetration_auxiliary = 0; // PENE_OLD(4), opaque to these phase helpers.
  double penetration_offset = 0;    // PENE_OLD(5).
  int irtlm[4]{};
  double time_s[2]{};
};
struct HistoryPhaseInput {
  double secondary_stiffness = 0, main_stiffness = 0;
  int local_processor = 0; // Native ISPMD+1. No source-ID special cases.
};
struct HistoryPhaseResult {
  NativeContactRow row;
  bool retained_candidate = false;
};
} // namespace tlfea::contact::radioss_type25
