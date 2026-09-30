// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <cstring>
namespace tl::fea::beam18::model_detail {
namespace { bool Same(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(a))==0;} }
bool SameMaterial(const Material& a,const Material& b) noexcept {
  if(a.material.native_units!=b.material.native_units||a.material.hardening!=b.material.hardening||
      a.curve.count!=b.curve.count) return false;
#define TL_SAME(field) if(!Same(a.field,b.field)) return false
  TL_SAME(material.young_pa); TL_SAME(material.poisson_ratio); TL_SAME(material.density_kg_m3);
  TL_SAME(material.rate_c_per_s); TL_SAME(material.rate_p); TL_SAME(material.cutoff_hz);
  TL_SAME(shear_pa); TL_SAME(twice_shear_pa); TL_SAME(three_shear_pa); TL_SAME(bulk_pa);
  TL_SAME(sound_speed_m_s); TL_SAME(inverse_rate_c); TL_SAME(inverse_rate_p);
  TL_SAME(angular_cutoff_per_s); TL_SAME(stress_limit_pa); TL_SAME(stress_floor_pa);
  TL_SAME(plastic_cap_strain); TL_SAME(failure_plastic_strain);
  TL_SAME(material.analytic.a_pa); TL_SAME(material.analytic.b_pa); TL_SAME(material.analytic.exponent);
  TL_SAME(material.analytic.maximum_stress_pa); TL_SAME(material.analytic.maximum_plastic_strain);
#undef TL_SAME
  const auto count=a.curve.count;
  return count&&std::memcmp(a.curve.plastic_strain,b.curve.plastic_strain,count*sizeof(double))==0&&
      std::memcmp(a.curve.yield_stress_pa,b.curve.yield_stress_pa,count*sizeof(double))==0;
}
bool SameReference(const Reference& a,const Reference& b) noexcept {
  if(!force_detail::SameReference(a,b)) return false;
  const auto& x=a.input(); const auto& y=b.input();
  return Same(x.radius,y.radius)&&Same(x.density,y.density)&&Same(x.young,y.young)&&Same(x.poisson,y.poisson);
}
bool CopyMaterial(const Material& input,double*& storage,Material& output) noexcept {
  const auto count=input.curve.count;
  double* strain=storage; storage+=count; double* stress=storage; storage+=count;
  std::memcpy(strain,input.curve.plastic_strain,count*sizeof(double));
  std::memcpy(stress,input.curve.yield_stress_pa,count*sizeof(double));
  return tl::material::law44::solid::Prepare(input.material,{strain,stress,count},output)==
      tl::material::law44::solid::Status::Ok&&SameMaterial(input,output);
}
} // namespace tl::fea::beam18::model_detail
