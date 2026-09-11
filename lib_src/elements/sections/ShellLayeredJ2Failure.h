// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected local MULAWC / FAIL_SETOFF_C caller, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "ShellLayeredJ2Work.h"
#include "lib_src/materials/failure/ShellConstantPlasticFailure.h"

namespace tl::fea::sections {
using ConstantFailureParameters=tl::material::failure::ConstantPlasticFailureParameters;
using ConstantFailureHistory=tl::material::failure::ConstantPlasticFailureHistory;

struct ShellFailureForcePoint { double stress[5]{}; };
struct ShellLayeredJ2FailureHistory {
  ShellLayeredJ2History saved{}; // SIGOFF-masked stress; PLA/rate remain native.
  ConstantFailureHistory failure[3]{};
  ShellFailureForcePoint current_force_point[3]{}; // Before point/parent masking.
  bool element_active=true;
};
struct ShellLayeredJ2FailureResult {
  ShellLayeredJ2FailureHistory history{};
  // Current unmasked point results, final parent-masked material resultants.
  // Use history.saved for the next material evaluation, never current.history.
  ShellLayeredJ2Result current{};
  double constitutive_increment[3]{},caller_failure_increment[3]{};
  bool removed_now=false;
};

// Exact resolved IFAIL_SH=2 / positive default P_Thick_Fail / centered NIP3.
// Other thresholds, multiple layers or failure models require separate admission.
TL_SHELL_SECTION_HD inline double ShellNip3FailureThreshold() noexcept {
  return 1.-1.e-6;
}
TL_SHELL_SECTION_HD inline double ShellNip3FailedThickness(
    const ConstantFailureHistory (&point)[3]) noexcept {
  double thickness=0;
  for(unsigned p=0;p<3;++p)
    if(!point[p].point_active) thickness=thickness+LayerForceWeight(p);
  return thickness;
}

TL_SHELL_SECTION_HD inline PointStatus UpdateShellLayeredJ2Failure(
    const PointParameters&,const ConstantFailureParameters&,
    const ShellLayeredJ2FailureHistory&,const ShellLayeredJ2Input&,
    double native_evaluation_time_s,ShellLayeredJ2FailureResult&) noexcept;
} // namespace tl::fea::sections

#include "ShellLayeredJ2FailureUpdate.h"
