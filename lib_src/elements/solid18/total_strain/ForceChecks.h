// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceTypes.h"
#include "Reference.h"
#include "lib_src/materials/law90/MaterialIdentity.h"
#include "lib_src/materials/law90/CallerChecks.h"
namespace tl::fea::solid18::total_strain::force_detail {
using tl::material::law90::identity_detail::SameBits;
TL_SOLID18_HD inline bool SameReference(const Reference& a,const Reference& b) noexcept {
  if(!a.prepared()||!b.prepared()||!detail::Supported(a.input().profile)||
     !detail::Supported(b.input().profile))return false;
  const auto& x=a.input();const auto& y=b.input();
  if(x.source_element_id!=y.source_element_id||x.source_part_id!=y.source_part_id||
     x.source_section_id!=y.source_section_id||x.source_material_id!=y.source_material_id||
     !SameBits(x.density_kg_m3,y.density_kg_m3))return false;
  for(unsigned n=0;n<8;++n) {
    if(x.source_node_id[n]!=y.source_node_id[n]||
       !SameBits(x.position_m[n].x,y.position_m[n].x)||
       !SameBits(x.position_m[n].y,y.position_m[n].y)||
       !SameBits(x.position_m[n].z,y.position_m[n].z))return false;
  }
  return true;
}
TL_SOLID18_HD inline bool ValidMaterial(const Reference& r,const Material& m) noexcept {
  return r.prepared()&&detail::Supported(r.input().profile)&&m.initialized()&&
      SameBits(r.input().density_kg_m3,m.reader().density_kg_m3)&&
      // Original blank RHOR resolves PM1=PM89. Reference has that one density.
      SameBits(r.input().density_kg_m3,m.reader().reference_density_kg_m3);
}
TL_SOLID18_HD inline bool ValidValues(const Material& m,const HistoryValues& h) noexcept {
  for(const auto& p:h.point)
    if(!tl::material::law90::caller_detail::ValidHistory(m,p))return false;
  for(double x:h.global.stress_pa)if(!tl::math::Finite(x))return false;
  return solid18::detail::Positive(h.global.density_kg_m3)&&
      tl::math::Finite(h.global.internal_energy_density_j_m3)&&
      tl::math::Finite(h.global.bulk_pressure_pa)&&h.global.bulk_pressure_pa>=0&&
      tl::math::Finite(h.global.scalar_rate_per_s)&&h.global.scalar_rate_per_s>=0;
}
TL_SOLID18_HD inline bool ValidInterval(const History& h,const PrescribedInterval& v) noexcept {
  if(!h.prepared()||!solid18::detail::Positive(v.dt_s)||
     !SameBits(h.stamp().time_s,v.base_time_s)||h.stamp().sample_index==UINT64_MAX||
     v.sample_index!=h.stamp().sample_index+1)return false;
  const double endpoint=v.base_time_s+v.dt_s;
  if(!tl::math::Finite(endpoint)||endpoint<=v.base_time_s)return false;
  for(unsigned n=0;n<8;++n)
    if(!solid18::detail::Finite(v.position_endpoint_m[n])||
       !solid18::detail::Finite(v.velocity_midpoint_m_s[n]))return false;
  return true;
}
} // namespace tl::fea::solid18::total_strain::force_detail
