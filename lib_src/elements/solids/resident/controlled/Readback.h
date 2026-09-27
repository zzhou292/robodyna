// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ProfiledResults.h"
#include "../DeviceFamilies.h"
#include "lib_src/elements/solid24/controlled_hourglass/UnitConversions.h"
namespace tl::fea::solids::batch_detail::controlled {
inline bool Selected(const Model& model,Family family,std::size_t local)noexcept {
  const auto* s=model.control_selection();if(!s||s->profile()!=control::Profile::SourceDeclared)return false;
  std::size_t index=local;
  if(family==Family::Solid24)index+=model.solid18().size();
  else if(family==Family::Solid18Law90)index+=model.solid18().size()+model.solid24().size()+model.solid6z().size()+model.solid18_law44().size();
  else return false;
  return index<s->parents().size()&&s->parents()[index].family==family&&s->parents()[index].family_index==local&&s->parents()[index].source.icontrol==1;
}
template<class MaterialHistory> inline MaterialObservationSI MaterialSI(const MaterialHistory& source,
    const solid_common::distortion::units_detail::Factors& f)noexcept {
  MaterialObservationSI result;
  for(unsigned k=0;k<6;++k)result.stress_pa[k]=source.stress_pa[k]*f.pressure;
  result.density_kg_m3=source.density_kg_m3*(f.base.mass/f.volume);
  result.internal_energy_density_j_m3=source.internal_energy_density_j_m3*f.pressure;
  result.bulk_pressure_pa=source.bulk_pressure_pa*f.pressure;return result;
}
inline ProfiledResult24 Read(const State<Traits24>& state)noexcept {
  ProfiledResult24 out;out.stamp=state.history.stamp();
  for(unsigned n=0;n<8;++n)out.cache.rhs_force_n[n]=state.cache.rhs_force_n[n];out.cache.stiffness=state.cache.stiffness;
  solid_common::distortion::units_detail::Factors f;
  if(const auto* native=state.history.native()) {
    out.history.SetNative({native->native_values(),native->native_distortion_energy()});out.history_units=native->units();
    solid_common::distortion::units_detail::Make(out.history_units,f);
    out.material_si=MaterialSI(native->native_values().material,f);out.cache.response=state.cache.controlled;
  } else {
    const auto& h=*state.history.legacy();out.history.SetLegacy(h.values());
    solid_common::distortion::units_detail::Make(out.history_units,f);out.material_si=MaterialSI(h.values().material,f);
    const auto& d=state.cache.diagnostics;
    out.cache.response={out.history_units,d.material.raw_stiffness_n_m,d.material.raw_stiffness_n_m,
      d.material.raw_stiffness_n_m,d.material.unscaled_element_dt_s,d.material.internal_work_j,d.stabilization_work_j,0,0};
  }
  return out;
}
inline ProfiledResult18Law90 Read(const State<Traits18Law90>& state)noexcept {
  ProfiledResult18Law90 out;out.stamp=state.history.stamp();
  for(unsigned n=0;n<8;++n)out.cache.rhs_force_n[n]=state.cache.rhs_force_n[n];out.cache.stiffness=state.cache.stiffness;
  const solid18::total_strain::HistoryValues* values=nullptr;
  if(const auto* native=state.history.native()) {
    values=&native->native_history().data();out.history.SetNative({*values,native->native_distortion_energy()});
    out.history_units=native->units();out.cache.response=state.cache.controlled;
  } else {
    values=&state.history.legacy()->data();out.history.SetLegacy(*values);const auto& d=state.cache.diagnostics;
    out.cache.response={out.history_units,d.raw_stiffness_n_m,0,0,d.minimum_unscaled_dt_s,d.internal_work_increment_j,0,0,0};
  }
  solid_common::distortion::units_detail::Factors f;solid_common::distortion::units_detail::Make(out.history_units,f);
  for(unsigned ip=0;ip<8;++ip)out.material_si[ip]=MaterialSI(values->point[ip],f);
  out.global_si=values->global;
  for(double& value:out.global_si.stress_pa)value*=f.pressure;
  out.global_si.density_kg_m3*=f.base.mass/f.volume;out.global_si.internal_energy_density_j_m3*=f.pressure;
  out.global_si.bulk_pressure_pa*=f.pressure;out.global_si.scalar_rate_per_s/=f.base.time;return out;
}
} // namespace tl::fea::solids::batch_detail::controlled
