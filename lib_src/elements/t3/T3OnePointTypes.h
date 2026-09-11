// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected C3FORC3/MULAWC packets; OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3History.h"
#include "T3ForceData.h"
#include "lib_src/elements/sections/ShellLaw44MembranePoint.h"

namespace tl::fea::t3 {
using OnePointMaterial=tl::material::TabulatedShellPlasticityParameters;
using OnePointFailure=tl::material::failure::ConstantPlasticFailureParameters;
using OnePointMaterialHistory=tl::material::TabulatedShellPlasticityHistory;
using OnePointFailureHistory=tl::material::failure::ConstantPlasticFailureHistory;

struct OnePointHistoryValues {
  HistoryValues shell; // Native FOR/MOM/GSTR/EINT/THK/global EPSD, OFF0/1.
  OnePointMaterialHistory point; // Exactly one genuine LBUF point.
  OnePointFailureHistory failure;
  double plastic_work_j=0; // Native WPLA, not an addition to EINT.
};
class OnePointHistory {
 public:
  TL_T3_HD bool prepared() const noexcept { return shell_.prepared(); }
  TL_T3_HD const History& shell() const noexcept { return shell_; }
  TL_T3_HD const OnePointMaterial& material() const noexcept { return material_; }
  TL_T3_HD OnePointFailure failure_parameters() const noexcept { return failure_parameters_; }
  TL_T3_HD const OnePointMaterialHistory& point() const noexcept { return point_; }
  TL_T3_HD const OnePointFailureHistory& failure() const noexcept { return failure_; }
  TL_T3_HD double plastic_work_j() const noexcept { return plastic_work_j_; }
  TL_T3_HD OnePointHistoryValues values() const noexcept {
    return {shell_.data(),point_,failure_,plastic_work_j_};
  }
 private:
  History shell_;
  OnePointMaterial material_;
  OnePointFailure failure_parameters_;
  OnePointMaterialHistory point_;
  OnePointFailureHistory failure_;
  double plastic_work_j_=0;
  friend TL_T3_HD Status PrepareOnePointLaw44History(const ReferenceData&,const OnePointMaterial&,
      OnePointFailure,const OnePointHistoryValues&,HistoryStamp,OnePointHistory&) noexcept;
};
struct OnePointForceTrial {
  OnePointHistory proposed_history;
  Kinematics kinematics; // Actual endpoint geometry and midpoint v/omega.
  double strain_curvature_increment[8]{}; // XX YY XY YZ ZX KXX KYY KXY.
  sections::MembraneLaw44PointResult point; // Current, saved, thickness and D1 channels.
  Vec3 internal_force[3]{},internal_couple[3]{}; // Positive native world N/N*m; RHS subtracts.
  ForceDiagnostics diagnostics; // Current-force THK0 and native STI/STIR/DTEL.
  double plastic_work_increment_j=0;
  bool removed_now=false;
};
static_assert(sizeof(OnePointHistory)<2048,"Bounded one-point prescribed history");
static_assert(sizeof(OnePointForceTrial)<4096,"Bounded one-point prescribed trial");
} // namespace tl::fea::t3
