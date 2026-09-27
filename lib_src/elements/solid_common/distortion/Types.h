// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law42/MechanicalSlots.h"
namespace tl::fea::solid_common::distortion {
// First profile: eight-node H24, SI, active LAW42 alpha2/no-Prony, ISMSTR10.
// No S6 geometric-distortion dispatch and no LAW90 STI/STIN contract is enabled.
inline constexpr double MaximumPoissonRatio=static_cast<double>(0.48999f);
enum class WorkingUnits { SI=1 };
enum class Status { Success, InvalidMaterial, InvalidInput, UnsupportedProfile, NonfiniteResult };
struct Input {
  // Native GBUF%SIG in its caller frame: XX,YY,ZZ,XY,YZ,ZX. Full Cauchy
  // components include hydrostatic stress and exclude artificial QVIS.
  double cauchy_stress_pa[6]{};
  double density_kg_m3=0, material_sound_speed_m_s=0, current_volume_m3=0;
  double off=1, offg=1;
  int ismstr=10;
  WorkingUnits units=WorkingUnits::SI;
};
struct Parameters {
  double length_m=0;             // LL, current volume ** THIRD.
  double damping_n_s_m2=0;        // FLD, kg/(m*s) = N*s/m^2.
  double control_stiffness_n_m=0;// STI_C, not assembled STI or LAW90 STIN.
  double damping_coefficient=0;  // MU / caller CNS2.
  double quadratic_limit=0;      // FQMAX.
  int buckling_flag=0;           // ISTAB from SCRE_SIG3 component minimum.
};
} // namespace tl::fea::solid_common::distortion
