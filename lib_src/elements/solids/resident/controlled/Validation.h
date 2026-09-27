// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ResultChecks.h"
#include "Storage.h"
namespace tl::fea::solids::batch_detail::controlled {
TL_BRICK_HD inline bool Units(solid_common::distortion::UnitScale a,solid_common::distortion::UnitScale b)noexcept {
  return a.length_m==b.length_m&&a.mass_kg==b.mass_kg&&a.time_s==b.time_s;
}
TL_BRICK_HD inline bool Observation(const ControlledObservation& c)noexcept {
  const double values[]{c.material_raw_stiffness_n_m,c.hourglass_raw_stiffness_n_m,c.after_distortion_raw_stiffness_n_m,
    c.material_dt_s,c.material_work_j,c.hourglass_work_j,c.distortion_energy_j,c.distortion_work_j};
  solid_common::distortion::units_detail::Factors f;
  return FiniteValues(values)&&solid_common::distortion::units_detail::Make(c.units,f)&&
    c.material_raw_stiffness_n_m>0&&c.after_distortion_raw_stiffness_n_m>0&&c.material_dt_s>0;
}
template<class H> TL_BRICK_HD inline bool MaterialSiFinite(const H& h,
    const solid_common::distortion::units_detail::Factors& f)noexcept {
  for(double v:h.stress_pa)if(!tl::math::Finite(v*f.pressure))return false;
  return tl::math::Finite(h.density_kg_m3*(f.base.mass/f.volume))&&
    tl::math::Finite(h.internal_energy_density_j_m3*f.pressure)&&tl::math::Finite(h.bulk_pressure_pa*f.pressure);
}
TL_BRICK_HD inline bool Valid(const h24::Reference& reference,const State<Traits24>& state,
    double time,std::uint64_t epoch)noexcept {
  const auto* h=state.history.native();const auto& c=state.cache;const auto& o=c.controlled;
  if(!h||!h->prepared()||c.profile!=ResultProfile::NativeControlled||!Observation(o)||
     h->stamp().time_s!=time||h->stamp().sample_index!=epoch||!Units(h->units(),reference.units())||!Units(o.units,h->units())||
     !solid24::force_detail::SameReference(h->reference(),reference.reference())||
     !solid24::force_detail::SameMaterial(h->material(),reference.material())||
     !Material42(h->native_values().material)||!Forces(c.rhs_force_n)||o.hourglass_raw_stiffness_n_m<=0||
     !Coefficients(c.stiffness,o.after_distortion_raw_stiffness_n_m,o.material_dt_s,.25))return false;
  for(const auto& row:h->native_values().controlled_hourglass.force_n)if(!FiniteValues(row))return false;
  solid_common::distortion::units_detail::Factors f;
  return solid_common::distortion::units_detail::Make(h->units(),f)&&MaterialSiFinite(h->native_values().material,f)&&tl::math::Finite(h->native_distortion_energy())&&
    o.distortion_energy_j==h->native_distortion_energy()*f.base.energy;
}
TL_BRICK_HD inline bool Valid(const foam::Reference& reference,const State<Traits18Law90>& state,
    double time,std::uint64_t epoch)noexcept {
  const auto* h=state.history.native();const auto& c=state.cache;const auto& o=c.controlled;
  if(!h||!h->prepared()||c.profile!=ResultProfile::NativeControlled||!Observation(o)||!Units(h->units(),reference.material().units())||
     !Units(o.units,h->units())||!Forces(c.rhs_force_n))return false;
  const auto& native=h->native_history();const auto& material=reference.material().native_material();
  if(NativeStamp(*h).time_s!=time||native.stamp().sample_index!=epoch||
     !solid18::total_strain::force_detail::SameReference(native.reference(),reference.native_reference())||
     !tl::material::law90::SamePreparedMaterial(native.material(),material)||
     !solid18::total_strain::force_detail::ValidValues(material,native.data())||
     c.diagnostics.raw_stiffness_n_m!=o.material_raw_stiffness_n_m||
     c.diagnostics.minimum_unscaled_dt_s!=o.material_dt_s||c.diagnostics.internal_work_increment_j!=o.material_work_j||
     o.hourglass_raw_stiffness_n_m!=0||o.hourglass_work_j!=0||
     !Coefficients(c.stiffness,c.diagnostics.raw_stiffness_n_m,o.material_dt_s,.25))return false;
  solid_common::distortion::units_detail::Factors f;
  if(!solid_common::distortion::units_detail::Make(h->units(),f)||!MaterialSiFinite(native.data().global,f))return false;
  for(const auto& point:native.data().point)if(!MaterialSiFinite(point,f))return false;
  return tl::math::Finite(h->native_distortion_energy())&&o.distortion_energy_j==h->native_distortion_energy()*f.base.energy;
}
} // namespace tl::fea::solids::batch_detail::controlled
