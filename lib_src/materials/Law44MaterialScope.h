// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TabulatedShellPlasticity.h"
#if defined(__CUDACC__)
#define TL_LAW44_SCOPE_HD __host__ __device__
#else
#define TL_LAW44_SCOPE_HD
#endif
namespace tl::material {
namespace law44_scope_detail {
TL_LAW44_SCOPE_HD inline bool SameBits(double a,double b) noexcept {
  const auto* x=reinterpret_cast<const unsigned char*>(&a);
  const auto* y=reinterpret_cast<const unsigned char*>(&b);
  for (unsigned i=0;i<sizeof(double);++i) if (x[i]!=y[i]) return false;
  return true;
}
} // namespace law44_scope_detail
// Complete immutable prepared-parameter scope; borrowed curve backing identity
// is intentional. The caller keeps those arrays immutable and alive.
TL_LAW44_SCOPE_HD inline bool SameLaw44Parameters(
    const TabulatedShellPlasticityParameters& a,const TabulatedShellPlasticityParameters& b) {
  if (a.hardening!=b.hardening || a.continuation!=b.continuation ||
      a.curve.count!=b.curve.count || a.curve.plastic_strain!=b.curve.plastic_strain ||
      a.curve.yield_stress_pa!=b.curve.yield_stress_pa ||
      a.rate.enabled!=b.rate.enabled || a.rate.policy!=b.rate.policy) return false;
  const double x[]{a.young_pa,a.poisson_ratio,a.density_kg_m3,a.shear_modulus,a.a11,a.a12,
      a.three_g,a.sound_speed,a.inverse_rate_c,a.inverse_rate_p,a.angular_cutoff_per_s,
      a.rate.cowper_symonds_c_per_s,a.rate.cowper_symonds_p,a.rate.cutoff_hz,
      a.linear.initial_yield_pa,a.linear.tangent_modulus_pa,a.plastic_hardening_pa};
  const double y[]{b.young_pa,b.poisson_ratio,b.density_kg_m3,b.shear_modulus,b.a11,b.a12,
      b.three_g,b.sound_speed,b.inverse_rate_c,b.inverse_rate_p,b.angular_cutoff_per_s,
      b.rate.cowper_symonds_c_per_s,b.rate.cowper_symonds_p,b.rate.cutoff_hz,
      b.linear.initial_yield_pa,b.linear.tangent_modulus_pa,b.plastic_hardening_pa};
  for (unsigned i=0;i<17;++i) if (!law44_scope_detail::SameBits(x[i],y[i])) return false;
  return true;
}
} // namespace tl::material
#undef TL_LAW44_SCOPE_HD
