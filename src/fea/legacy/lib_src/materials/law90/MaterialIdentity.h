// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tl::material::law90 {
namespace identity_detail {
TL_LAW90_HD inline bool SameBits(double a,double b) noexcept {
  const auto* x=reinterpret_cast<const unsigned char*>(&a);
  const auto* y=reinterpret_cast<const unsigned char*>(&b);
  for (unsigned k=0;k<sizeof(double);++k) if(x[k]!=y[k])return false;
  return true;
}
}
// Prepared values borrow one immutable curve lifetime. Pointer equality is an
// identity check, not an assertion that mutable or expired backing is safe.
TL_LAW90_HD inline bool SamePreparedMaterial(const PreparedMaterial& a,
                                             const PreparedMaterial& b) noexcept {
  if (!a.initialized()||!b.initialized()||a.curve().count!=b.curve().count||
      a.curve().compression_strain!=b.curve().compression_strain||
      a.curve().stress_pa!=b.curve().stress_pa) return false;
  const auto& x=a.reader();const auto& y=b.reader();
  const double av[]{x.density_kg_m3,x.reference_density_kg_m3,x.card_young_pa,
      x.initial_shear_pa,x.poisson_ratio,x.contact_modulus_pa,x.contact_bulk_pa,
      x.tension_cutoff_pa,x.hysteresis,x.shape,x.alpha,x.curve_scale,
      x.curve_rate_s_inverse,x.filter_cutoff_hz};
  const double bv[]{y.density_kg_m3,y.reference_density_kg_m3,y.card_young_pa,
      y.initial_shear_pa,y.poisson_ratio,y.contact_modulus_pa,y.contact_bulk_pa,
      y.tension_cutoff_pa,y.hysteresis,y.shape,y.alpha,y.curve_scale,
      y.curve_rate_s_inverse,y.filter_cutoff_hz};
  for (unsigned k=0;k<14;++k) if(!identity_detail::SameBits(av[k],bv[k]))return false;
  if (x.smooth!=y.smooth||x.rate_flag!=y.rate_flag||x.loading_flag!=y.loading_flag||
      x.damage_flag!=y.damage_flag||x.tension_flag!=y.tension_flag||
      x.failure_mode!=y.failure_mode||x.material_viscosity_flag!=y.material_viscosity_flag||
      x.history_count!=y.history_count||x.cursor_count!=y.cursor_count) return false;
  const auto& u=a.updated();const auto& v=b.updated();
  const double au[]{u.minimum_curve_slope_pa,u.maximum_curve_slope_pa,u.initial_curve_slope_pa,
      u.average_curve_slope_pa,u.young_pa,u.shear_pa,u.bulk_pa,u.maximum_modulus_pa,
      u.hourglass_modulus_pa,u.maximum_strain};
  const double bu[]{v.minimum_curve_slope_pa,v.maximum_curve_slope_pa,v.initial_curve_slope_pa,
      v.average_curve_slope_pa,v.young_pa,v.shear_pa,v.bulk_pa,v.maximum_modulus_pa,
      v.hourglass_modulus_pa,v.maximum_strain};
  for (unsigned k=0;k<10;++k) if(!identity_detail::SameBits(au[k],bu[k]))return false;
  return true;
}
} // namespace tl::material::law90
