// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"

namespace tl::fea::solids::model_detail {
bool Same(const solid18::Material& a,const solid18::Material& b) noexcept {
  if(a.curve.count!=b.curve.count || !Same(a.young_pa,b.young_pa) ||
      !Same(a.poisson_ratio,b.poisson_ratio) || !Same(a.density_kg_m3,b.density_kg_m3) ||
      !Same(a.shear_pa,b.shear_pa) || !Same(a.twice_shear_pa,b.twice_shear_pa) ||
      !Same(a.three_shear_pa,b.three_shear_pa) || !Same(a.bulk_pa,b.bulk_pa) ||
      !Same(a.sound_speed_m_s,b.sound_speed_m_s))return false;
  for(std::size_t i=0;i<a.curve.count;++i)
    if(!Same(a.curve.plastic_strain[i],b.curve.plastic_strain[i]) ||
        !Same(a.curve.yield_stress_pa[i],b.curve.yield_stress_pa[i]))return false;
  return true;
}
bool Same(const solid24::Material& a,const solid24::Material& b) noexcept {
  return Same(a.mu_pa,b.mu_pa) && Same(a.poisson_ratio,b.poisson_ratio) &&
      Same(a.bulk_pa,b.bulk_pa) && Same(a.density_kg_m3,b.density_kg_m3) &&
      Same(a.tension_cutoff_pa,b.tension_cutoff_pa);
}
bool Same(const solid6z::ForceProfile& a,const solid6z::ForceProfile& b) noexcept {
  return a.stabilization==b.stabilization && Same(a.damping_coefficient,b.damping_coefficient) &&
      a.engine_frame==b.engine_frame && a.integration_control==b.integration_control &&
      a.compressibility_control==b.compressibility_control &&
      a.degenerate_step_control==b.degenerate_step_control;
}
} // namespace tl::fea::solids::model_detail
