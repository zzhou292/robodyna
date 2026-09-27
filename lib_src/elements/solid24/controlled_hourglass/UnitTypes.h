// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_src/collision/radioss_type25/coefficients/UnitFactors.h"
namespace tl::fea::solid24::controlled_hourglass {
using UnitScale=tlfea::contact::radioss_type25::UnitScale;
struct NativeModalWork {
  double rate[3][4]{},force[3][4]{},work=0;
  UnitScale units{}; // Native numerical values, not SI-labelled observations.
};
struct WorkingResult {
  ForceGeometry geometry;tl::material::law42::CallerResult material;
  BeforeDistortionResult stage;NativeModalWork native_modal_work;
};
class WorkingReference;
TL_BRICK_HD ForceStatus PrepareWorkingReference(const Reference&,const Material&,UnitScale,WorkingReference&) noexcept;
TL_BRICK_HD ForceStatus EvaluateWorking(const WorkingReference&,const HistoryValues&,
    const PrescribedInterval&,bool,WorkingResult&) noexcept;
class WorkingReference {
 public:
  TL_BRICK_HD bool prepared()const noexcept{return prepared_;}
  TL_BRICK_HD const Reference& reference()const noexcept{return physical_reference_;}
  TL_BRICK_HD const Material& material()const noexcept{return physical_material_;}
  TL_BRICK_HD UnitScale units()const noexcept{return units_;}
 private:
  Reference physical_reference_,numerical_reference_;
  Material physical_material_,numerical_material_;
  UnitScale units_{};bool prepared_=false;
  friend TL_BRICK_HD ForceStatus PrepareWorkingReference(const Reference&,const Material&,UnitScale,WorkingReference&) noexcept;
  friend TL_BRICK_HD ForceStatus EvaluateWorking(const WorkingReference&,const HistoryValues&,
      const PrescribedInterval&,bool,WorkingResult&) noexcept;
};
} // namespace tl::fea::solid24::controlled_hourglass
