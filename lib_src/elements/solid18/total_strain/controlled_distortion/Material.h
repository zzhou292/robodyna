// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law90/MechanicalSlots.h"
#include "lib_src/materials/law90/Prepare.h"
#include "lib_src/materials/law90/Relocate.h"
#include "lib_src/elements/solid_common/distortion/UnitTypes.h"
namespace tl::fea::solid18::total_strain::controlled_distortion {
namespace distortion=tl::fea::solid_common::distortion;
namespace law=tl::material::law90;
// Native-number material slots are prepared once, never recomputed per step.
// The immutable borrowed curve retains the same lifetime as the mechanics model.
class Material {
 public:
  TL_LAW90_HD bool prepared()const noexcept{return prepared_;}
  TL_LAW90_HD const law::PreparedMaterial& source()const noexcept{return source_;}
  TL_LAW90_HD const law::PreparedMaterial& native_material()const noexcept{return numeric_;}
  TL_LAW90_HD const law::MechanicalSlots& slots()const noexcept{return slots_;}
  TL_LAW90_HD distortion::UnitScale units()const noexcept{return units_;}
 private:
  law::PreparedMaterial source_,numeric_;
  law::MechanicalSlots slots_;
  distortion::UnitScale units_{};
  bool prepared_=false;
  friend TL_LAW90_HD distortion::Status RelocateMaterial(const Material&,law::CurveView,Material&) noexcept;
  friend TL_LAW90_HD distortion::Status PrepareMaterial(const law::PreparedMaterial&,
      distortion::UnitScale,Material&) noexcept;
};
TL_LAW90_HD inline distortion::Status PrepareMaterial(const law::PreparedMaterial& source,
    distortion::UnitScale units,Material& output) noexcept {
  distortion::units_detail::Factors factors;
  if(!source.initialized())return distortion::Status::InvalidInput;
  if(!distortion::units_detail::Make(units,factors))return distortion::Status::UnsupportedProfile;
  auto numeric=source;
  if(units.length_m!=1) {
    const auto& r=source.reader();law::PreparationInput input;
    // PreparedMaterial retains resolved TCUT, not whether EP20 came from a blank
    // native field. Unit conversion of that literal is ambiguous without source
    // provenance. The selected radiator has an explicit 15 MPa cutoff.
    if(r.tension_cutoff_pa==1e20)return distortion::Status::UnsupportedProfile;
    const double density=factors.base.mass/factors.volume;
    input.density_kg_m3=r.density_kg_m3/density;
    input.reference_density_kg_m3=r.reference_density_kg_m3/density;
    input.card_young_pa=r.card_young_pa/factors.pressure;
    input.poisson_ratio=r.poisson_ratio;
    input.contact_modulus_pa=r.contact_modulus_pa/factors.pressure;
    input.tension_cutoff_pa=r.tension_cutoff_pa/factors.pressure;
    input.hysteresis=r.loading_flag==1?0:r.hysteresis;
    input.shape=r.shape;input.alpha=r.alpha;
    input.curve_scale=r.curve_scale/factors.pressure;
    input.curve_scale_dimension=1;input.smooth=r.smooth;
    input.tension_flag=r.tension_flag;input.failure_mode=r.failure_mode;
    if(law::PrepareSI(input,source.curve(),numeric)!=law::Status::Ok)
      return distortion::Status::UnsupportedProfile;
  }
  Material next;
  if(law::PrepareMechanicalSlots(numeric,next.slots_)!=law::Status::Ok)
    return distortion::Status::NonfiniteResult;
  next.source_=source;next.numeric_=numeric;next.units_=units;next.prepared_=true;
  output=next;return distortion::Status::Success;
}
// Rebase immutable curve views during upload; never dereference device pointers here.
TL_LAW90_HD inline distortion::Status RelocateMaterial(const Material& source,
    law::CurveView curve,Material& output) noexcept {
  if(!source.prepared())return distortion::Status::InvalidInput;
  Material next=source;
  if(law::RelocatePreparedCurve(source.source_,curve,next.source_)!=law::Status::Ok||
     law::RelocatePreparedCurve(source.numeric_,curve,next.numeric_)!=law::Status::Ok)
    return distortion::Status::InvalidInput;
  output=next;return distortion::Status::Success;
}
} // namespace tl::fea::solid18::total_strain::controlled_distortion
