// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
namespace tlfea::contact::radioss_type25 {
enum class NormalStatus { Ok, InvalidInput, UnsupportedProfile, NonfiniteResult };
// Different types prevent accidental use of SI values in native-unit algebra.
struct NativeUnitsTag {};
struct SiUnitsTag {};
struct EngineControls {
  int kdtint = -1;
  int idtmins = -1;
  int idtmins_int = -1;
};
struct ResolvedNormalConfig {
  // P1 admits only this selected source branch, not a complete TYPE25 profile.
  int stiffness_formulation = 4;
  int damping_flag = 1;
  int initial_penetration = 5;
  int arithmetic_precision = 8;
  bool prescribed_contact_force = false;
  bool adhesion = false;
  double damping_factor = 0.05;
  EngineControls engine;
};
template<class Units> struct NormalInput {
  double penetration = 0; // After native geometry/initial-offset treatment.
  double stiffness = 0;   // Incoming STIF, before history, HALF and damping.
  double normal_velocity = 0;
  double dt = 0, time = 0; // dt is native DT1 (history interval), NOT proposed DT2.
  double secondary_mass = 0;
  double main_mass[4]{};
  double weights[4]{};
  double friction_viscosity = 0; // Native VISCFFRIC, not Coulomb mu.
};
template<class Units> struct NormalHistory {
  double previous_penetration = 0; // PENE_OLD(2)
  double previous_stiffness = 0;   // STIF_OLD(2)
  double staged_penetration = 0;   // PENE_OLD(1), max-accumulated at TT=0.
  double staged_stiffness = 0;     // STIF_OLD(1), BEFORE HALF/damping.
  double damping_half_force = 0;   // PENE_OLD(3), signed HALF*FF.
};
template<class Units> struct NormalResult {
  NormalHistory<Units> history;
  double weights[4]{};
  double force_stiffness = 0; // K used for the elastic signed normal force.
  double stability_stiffness = 0; // Final STIF; not a global dt certificate.
  double normal_force = 0; // Native FNI sign, including damping; no extra clamp.
  double elastic_energy = 0;
  double damping_force = 0; // Native FF=C*VN.
  double damping_work = 0;  // This call's ECONTDT increment, donor operation order.
  double damping_coefficient = 0;
  double separate_elastic_stiffness = 0; // Native KT, valid only when terms_valid.
  double separate_friction_damping = 0;  // Native CF, same validity.
  bool terms_valid = false;
};
using NativeNormalInput = NormalInput<NativeUnitsTag>;
using NativeNormalHistory = NormalHistory<NativeUnitsTag>;
using NativeNormalResult = NormalResult<NativeUnitsTag>;
using SiNormalInput = NormalInput<SiUnitsTag>;
using SiNormalHistory = NormalHistory<SiUnitsTag>;
using SiNormalResult = NormalResult<SiUnitsTag>;
} // namespace tlfea::contact::radioss_type25
